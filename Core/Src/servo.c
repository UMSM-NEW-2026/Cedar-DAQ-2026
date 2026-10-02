#include "servo.h"

void Servo_Init(Servo *s, TIM_TypeDef *timer, volatile uint32_t *ccr, int16_t initial, int16_t step, uint32_t interval_ms)
{
    s->timer       = timer;
    s->ccr         = ccr;
    s->current     = initial;
    s->target      = initial;
    s->step        = step;
    s->interval_ms = interval_ms;
    s->last_tick   = 0;
    s->state       = SERVO_IDLE;

    *s->ccr = initial;
}

void Servo_SetTarget(Servo *s, int16_t target)
{
    s->target = target;
    s->state  = SERVO_MOVING;
}

void Servo_Update(Servo *s)
{
    if (s->state == SERVO_IDLE) return;

    uint32_t curr_tick = HAL_GetTick();  // capture ONCE
    if (curr_tick - s->last_tick < s->interval_ms) return;

    s->last_tick = curr_tick;  // use the captured value

    if (s->current < s->target)
    {
        s->current += s->step;
        if (s->current > s->target) s->current = s->target;
    }
    else if (s->current > s->target)
    {
        s->current -= s->step;
        if (s->current < s->target) s->current = s->target;
    }

    *s->ccr = s->current;

    if (s->current == s->target)
        s->state = SERVO_IDLE;
}

void Servo_Update_Throttle(Servo *s, uint16_t step){
if(s->state == SERVO_IDLE) return;

uint32_t curr_tick = HAL_GetTick();  // capture ONCE
if (curr_tick - s->last_tick < s->interval_ms) return;

s->last_tick = curr_tick;  // use the captured value

if (s->current < s->target)
{
s->current += step;
if (s->current > s->target) s->current = s->target;
}
else if (s->current > s->target)
{
s->current -= step;
if (s->current < s->target) s->current = s->target;
}

*s->ccr = s->current;

if (s->current == s->target)
s->state = SERVO_IDLE;

}

uint8_t Servo_IsIdle(Servo *s)
{
    return s->state == SERVO_IDLE;
}
