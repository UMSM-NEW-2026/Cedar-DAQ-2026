/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stdio.h"
#include "servo.h"

//static void MX_NVIC_Init(void){
//	HAL_NVIC_SetPriorty(EXTI3_IRQn,0 ,0);
//	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
//}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */


// Servo pulse width bounds (in timer ticks, period=1999 → 0-1999)
// Standard servo: 1ms (low) to 2ms (high) pulse, on a 20ms period
// 1ms/20ms * 2000 = 100 ticks, 2ms/20ms * 2000 = 200 ticks
#define SERVO_MIN  2000
#define SERVO_MAX  4500
#define ADC_MIN  600 // ADC value of throttle not depressed at all
#define ADC_MAX  3301 // ADC value of throttle depressed all the way

//LCD code

#define LCD_CS_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_12

#define LCD_RS_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_3

#define LCD_CS_LOW()  (LCD_CS_GPIO_Port->BSRR = (uint32_t)LCD_CS_Pin << 16U)
#define LCD_CS_HIGH() (LCD_CS_GPIO_Port->BSRR = LCD_CS_Pin)

#define LCD_RS_CMD()  (LCD_RS_GPIO_Port->BSRR = (uint32_t)LCD_RS_Pin << 16U)
#define LCD_RS_DATA() (LCD_RS_GPIO_Port->BSRR = LCD_RS_Pin)

/* ===== GPS UART receive (NMEA) ===== */

static uint8_t gps_rx_byte;

static char nmea_line[128];
static volatile uint16_t nmea_idx = 0;
static volatile uint8_t nmea_ready = 0;

/* Latest parsed fix (decimal degrees) */
volatile int32_t gps_lat_deg   = 0;
volatile int32_t gps_lon_deg   = 0;
volatile double  gps_speed_mph = 0;   // speed over ground, mph
volatile int32_t exp_time = 0;
volatile int    gps_fix_valid = 0;
int32_t last_log_ms = 0;
int32_t diff = 0;

//volatile int32_t a = 1;
//volatile int32_t b = 0;
//volatile int32_t = 39792082;

volatile int32_t a = -1;
volatile int32_t b = 0;
volatile int32_t c = -42293358;
int8_t csv_row_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

static void parse_rmc_line(const char *s);
static double nmea_to_decimal(double ddmm_mmmm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void sd_log_gps(int32_t lat, int32_t lon, double speed)
{
    FIL  f;
    UINT bw;
    char row[96];

    int len = snprintf(row, sizeof(row),
                       "%lu,%.6f,%.6f,%.3f\r\n",
                       (unsigned long)HAL_GetTick(),
                       lat, lon, (double)speed);

    if (len <= 0 || len >= (int)sizeof(row)) return;

    if (f_open(&f, "0:/gps_log.csv", FA_WRITE | FA_OPEN_APPEND) == FR_OK)
    {
        f_write(&f, row, (UINT)len, &bw);
        f_close(&f);   // flush to card on every write
    }
}

	int id = 0;

	int checkpoint_passed[8] = {0};
	int next_checkpoint = 0;
	int lap = 1;
	int curr_time = 0;
	int prev_time = 0;

/* ===== NMEA helpers ===== */

static double nmea_to_decimal(double ddmm_mmmm)
{
    double degrees = floor(ddmm_mmmm / 100.0);
    double minutes = ddmm_mmmm - degrees * 100.0;
    return degrees + (minutes / 60.0);
}

/*
 * Parses $GPRMC or $GNRMC sentence.
 * Field layout (comma-separated, 0-based):
 *  0  = $GPRMC
 *  1  = hhmmss.sss
 *  2  = A (valid) / V (void)
 *  3  = ddmm.mmmm  (latitude)
 *  4  = N / S
 *  5  = dddmm.mmmm (longitude)
 *  6  = E / W
 *  7  = speed over ground, knots   ← was missing
 *  8  = track angle, degrees
 *  9  = date ddmmyy
 */
static void parse_rmc_line(const char *s)
{
  // $GPRMC,hhmmss.sss,A,llll.ll,a,yyyyy.yy,a, ... *CS
  // Fields (0-based):
  // 0=$GPRMC
  // 1=time
  // 2=status A/V
  // 3=lat
  // 4=N/S
  // 5=lon
  // 6=E/W

  char buf[128];
  strncpy(buf, s, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char *save = NULL;
  char *tok = strtok_r(buf, ",", &save);

  int field = 0;
  char status = 'V';
  double lat_raw = 0.0, lon_raw = 0.0;
  char ns = 'N', ew = 'E';
  double speed = 0.0;

  while (tok)
  {
    if (field == 2 && tok[0]) status = tok[0];
    if (field == 3 && tok[0]) lat_raw = atof(tok);
    if (field == 4 && tok[0]) ns = tok[0];
    if (field == 5 && tok[0]) lon_raw = atof(tok);
    if (field == 6 && tok[0]) ew = tok[0];
    if (field == 7 && tok[0]) speed = tok[0];

    tok = strtok_r(NULL, ",", &save);
    field++;
  }

  if (status == 'A' && lat_raw > 0.0 && lon_raw > 0.0)
  {
    double lat = nmea_to_decimal(lat_raw);
    double lon = nmea_to_decimal(lon_raw);

    if (ns == 'S') lat = -lat;
    if (ew == 'W') lon = -lon;

    gps_lat_deg = 1000000 * lat;
    gps_lon_deg = 1000000 * lon;
    gps_speed_mph = speed * 1.15;
    gps_fix_valid = 1;
    printf("=========== parsing ======== \r\n");
    printf("\r\nlat %.6f, lon: %.6f", lat, lon);
    printf("\r\nlat %d, lon: %d", gps_lat_deg, gps_lon_deg, gps_speed_mph);
  }
  else
  {
    gps_fix_valid = 0;
  }
}

static void gps_process_line_if_ready(void)
{
    if (!nmea_ready) return;
    nmea_ready = 0;

    if (strncmp(nmea_line, "$GPRMC", 6) == 0 || strncmp(nmea_line, "$GNRMC", 6) == 0)
    {
        parse_rmc_line(nmea_line);

        if (gps_fix_valid)
        {
        	printf("\r\n=========== gps valid ======== \r\n");
            printf("LAT: %.6f  LON: %.6f\r\n", gps_lat_deg, gps_lon_deg, gps_speed_mph);

            uint32_t now = HAL_GetTick();
            if (now - last_log_ms >= 1000)
            {
                sd_log_gps(gps_lat_deg, gps_lon_deg, gps_speed_mph);
            	last_log_ms = now;
            }
            printf("VALUES ARE A: %d, B: %d, C: %d\r\n", a,b,c);
            char csv_data[128];
            char *cols[9];

            if (a * gps_lat_deg + b * gps_lon_deg > c)
            {
                printf("\n\n ========== \r\n crossed %d checkpoint \r\n", csv_row_counter);

                FRESULT res = read_next_row("0:/trial.csv", csv_data, sizeof(csv_data));
                if (res == FR_OK)
                {
                    csv_row_counter++;

                    split_csv(csv_data, cols, 9);
                    a = atoi(cols[1]);
                    b = atoi(cols[3]);
                    c = atoi(cols[5]);
                    exp_time = atoi(cols[8]);
                    int32_t a = HAL_GetTick();

                    diff = (a <= exp_time) ? (exp_time - a) : (a - exp_time);
                    printf("=========== readed from csv ======== \r\n");
                    printf("curr time: %d, expected time: %d, diff: %d", a, exp_time, diff);
                    printf("new values: a: %d, b %d, c:%d\r\n", a, b, c);
                }
                else
                {
                    printf("RUHROH");
                }
            }
        }
    }
}

static void gps_start_receive(void)
{
  nmea_idx = 0;
  nmea_ready = 0;
  HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1);
}

static void sd_log_init(void)
{
    FIL   f;
    UINT  bw;
    /* FA_OPEN_APPEND creates the file if absent, otherwise appends */
    if (f_open(&f, "0:/gps_log.csv", FA_WRITE | FA_OPEN_APPEND) == FR_OK)
    {
        /* Write header only when file is empty (size == 0) */
        if (f_size(&f) == 0)
        {
            const char *hdr = "timestamp_ms,lat_deg,lon_deg,speed_kts\r\n";
            f_write(&f, hdr, strlen(hdr), &bw);
        }
        f_close(&f);
    }
}








void LCD_Write(uint8_t data, uint8_t is_data) {

    if (is_data) {
        LCD_RS_DATA();
    } else {
        LCD_RS_CMD();
    }


    LCD_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);
    LCD_CS_HIGH();
}

void LCD_Init(void) {
    HAL_Delay(100); // Wait for VCC to stabilize

    // Manual Reset Sequence
    LCD_Write(0x30, 0);
    HAL_Delay(5);
    LCD_Write(0x30, 0);
    HAL_Delay(1);
    LCD_Write(0x30, 0);
    HAL_Delay(1);

    LCD_Write(0x38, 0); // 2 lines, 5x8 font
    LCD_Write(0x08, 0); // Display OFF
    LCD_Write(0x01, 0); // Clear
    HAL_Delay(2);
    LCD_Write(0x06, 0); // Entry mode
    LCD_Write(0x0C, 0); // Display ON, Cursor OFF
}
void LCD_SetCursor(uint8_t col, uint8_t row) {

    const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};

    if (row > 3) row = 3;
    if (col > 19) col = 19;

    LCD_Write(0x80 | (col + row_offsets[row]), 0);
}

void LCD_Print(const char *str) {
    while (*str) {
        LCD_Write((uint8_t)(*str), 1);
        str++;
    }
}

Servo throttle, wiper, shifter;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // Start GPS UART reception (NMEA)
  HAL_Delay(500);
  sd_mount();
  gps_start_receive();
  sd_log_init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //throttle
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //shifter
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); //wiper

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  Servo_Init(&throttle, TIM2, &TIM2->CCR2, 2000, 10, 5);
  Servo_Init(&wiper, TIM15, &TIM15->CCR2, 2000, 10, 6);
  Servo_Init(&shifter, TIM2, &TIM1->CCR1, 2000, 10, 3);
  HAL_Delay(500);
  LCD_Init();

  LCD_SetCursor(0, 0);
  LCD_Print("SPEED: ");

  LCD_SetCursor(0, 1);
  LCD_Print("DIFF: ");

  LCD_SetCursor(0, 2);
  LCD_Print("CHECKPOINT: ");

  LCD_SetCursor(0, 3);
  LCD_Print("LAP NUM 0/4");

  uint8_t wiper_dir = 1;
//  HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1);
  uint32_t last_log_ms = 0;
    //NEED TO CHANGE BASED ON WHICH TRACK WE'RE USING!!!
//  int8_t a = -1;
//  int8_t b = 0;
//  int32_t c = -42293358;
  uint32_t step;

  //sets E2 to low
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  char speed_char[8]; // Increased size to fit "15.123" + null
	  char diff_char[8]; // Increased
	  // %.3f specifies 3 digits after the decimal point
	  snprintf(speed_char, sizeof(speed_char), "%.1f", gps_speed_mph);
	  LCD_SetCursor(0, 0);
	  LCD_Print("SPEED: ");
	  LCD_SetCursor(7, 0);
	  LCD_Print(speed_char);
	  LCD_SetCursor(0, 1);

	  LCD_Print("DIFF: ");
	  int16_t seconds = 223370.67 * 0.0001;
	  snprintf(diff_char, sizeof(diff_char), "%.1f", seconds);
	  LCD_SetCursor(7, 1);
	  LCD_Print(speed_char);

	  LCD_SetCursor(0, 2);
	  LCD_Print("CHECKPOINT: ");


	  LCD_SetCursor(0, 3);
	  LCD_Print("LAP NUM 0/4");
	  HAL_ADC_Start(&hadc1);
	  //	  Servo_Update_Throttle(&throttle, step);
		Servo_Update(&throttle);
		Servo_Update(&wiper);
		Servo_Update(&shifter);
//		char[128] buf2 = nmea_line;
		char buf2[128];
		strcpy(buf2, nmea_line);
//		parse_rmc_line(nmea_line);
		gps_process_line_if_ready();


//		  printf("LAT=%.6f  LON=%.6f  SPD=%.1f kts\r\n",
//				 gps_lat_deg, gps_lon_deg, (double)gps_speed_kts);

		  	  //SIMON CHANGE THE TARGET VALUES IF THE SERVO ISNT SPINNING ENOUGH
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == 1){
		  Servo_SetTarget(&shifter, 6000);
		}
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == 0){
				  Servo_SetTarget(&shifter, 3000);
		}
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK){
			uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			uint32_t clamped = adc_value < ADC_MIN ? ADC_MIN :
							   adc_value > ADC_MAX ? ADC_MAX : adc_value;

//			uint32_t pulse = SERVO_MAX - ((clamped - ADC_MIN) * (SERVO_MAX - SERVO_MIN))
//										  / (ADC_MAX - ADC_MIN);
			uint32_t pulse = SERVO_MIN + ((clamped - ADC_MIN) * (SERVO_MAX - SERVO_MIN))
									  / (ADC_MAX - ADC_MIN);
			step = (SERVO_MAX - clamped)/500;
			Servo_SetTarget(&throttle, pulse);
		}
		else{
			HAL_ADC_Stop(&hadc1);  // stop even on timeout
		}

		if(Servo_IsIdle(&wiper)){
		  //HAL_Delay(10);
		  if(wiper_dir == 0){

			  //we cchanged from 7000 and 4000 to these numbers, wiper is going too far off on the left side
			  //facing out from in
			  Servo_SetTarget(&wiper, 6750);
			  wiper_dir = 1;
		  }else{
			  Servo_SetTarget(&wiper, 4250);
			  wiper_dir = 0;
		  }
		}





//     Optional: if you have printf routed somewhere, you can print when valid
//     (GPS typically updates at ~1 Hz)




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  sd_unmount();
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 3;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 62499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 39999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PG0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE11 PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//  if (huart->Instance == huart3.Instance) {
//    char c = (char)gps_rx_byte;
//    if (!nmea_ready) {
//      if (c == '\n') { // strip CR if present
//        if (nmea_idx > 0 && nmea_line[nmea_idx - 1] == '\r')
//          nmea_idx--; nmea_line[nmea_idx] = '\0';
//          nmea_ready = 1; nmea_idx = 0;
//        } else {
//          if (nmea_idx < (sizeof(nmea_line) - 1)){
//            nmea_line[nmea_idx++] = c;
//          }
//        else nmea_idx = 0; // overflow reset
//        }
//      } // Re-arm interrupt reception for next byte
//      HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1);
//    }
//}

/**
  * @brief Rx Transfer completed callback.
  * This assembles NMEA lines terminated by '\n'.
  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  if (huart->Instance == huart3.Instance)
//  {
//    // 1. RE-ARM IMMEDIATELY
//    // This minimizes the window where the receiver is "off"
//    uint8_t next_byte = gps_rx_byte;
//    HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1);
//
//    // 2. Now process the byte we just caught
//    char c = (char)next_byte;
//
//    if (!nmea_ready)
//    {
//      if (c == '\n')
//      {
//        if (nmea_idx > 0 && nmea_line[nmea_idx - 1] == '\r')
//          nmea_idx--;
//
//        nmea_line[nmea_idx] = '\0';
//        nmea_ready = 1;
//        nmea_idx = 0;
//      }
//      else
//      {
//        if (nmea_idx < (sizeof(nmea_line) - 1))
//        {
//          nmea_line[nmea_idx++] = c;
//        }
//        else
//        {
//          nmea_idx = 0; // Buffer safety
//        }
//      }
//    }
//  }
//}
/**
  * @brief UART error callback (helps recover from framing/overrun errors)
  */
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//  if (huart->Instance == huart3.Instance)
//  {
//    HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1);
//  }
//}


/**
  * @brief Rx Transfer completed callback.
  * This assembles NMEA lines terminated by '\n'.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart3.Instance)
  {
    char c = (char)gps_rx_byte;

    if (!nmea_ready)
    {
      if (c == '\n')
      {
        // strip CR if present
        if (nmea_idx > 0 && nmea_line[nmea_idx - 1] == '\r')
          nmea_idx--;

        nmea_line[nmea_idx] = '\0';
        nmea_ready = 1;
        nmea_idx = 0;
      }
      else
      {
        if (nmea_idx < (sizeof(nmea_line) - 1))
          nmea_line[nmea_idx++] = c;
        else
          nmea_idx = 0; // overflow reset
      }
    }

    // Re-arm interrupt reception for next byte
    HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1);
  }
}

/**
  * @brief UART error callback (helps recover from framing/overrun errors)
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart3.Instance)
  {
    HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1);
  }
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM5)  // your 1-second timer
//    {
//        rpm = pulse_count * 60;  // pulses per second → per minute
//        printf("RPM: %d \r\n", rpm);
//        pulse_count = 0;         // reset for next interval
//    }
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
