/*
 * servo.h
 *
 *  Created on: Mar 28, 2026
 *      Author: varunkaushik
 */

#ifndef SERVO_H
#define SERVO_H

#include "stm32l4xx_hal.h"  // adjust for your MCU
#include <stdint.h>

typedef enum {
    SERVO_IDLE,
    SERVO_MOVING
} ServoState;

typedef struct {
    TIM_TypeDef  *timer;
    uint32_t      *ccr;   // e.g. pointer offset — just store CCR directly
    int16_t       current;
    int16_t       target;
    int16_t       step;
    uint32_t      interval_ms;
    uint32_t      last_tick;
    ServoState    state;
} Servo;

void Servo_Init(Servo *s, TIM_TypeDef *timer, volatile uint32_t *ccr, int16_t initial, int16_t step, uint32_t interval_ms);
void Servo_SetTarget(Servo *s, int16_t target);
void Servo_Update(Servo *s);
uint8_t Servo_IsIdle(Servo *s);

#endif
