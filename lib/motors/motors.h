#pragma once

#include "stm32f4xx_hal.h"
#include "pid.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim_fwd;   /* таймер RPWM (вперёд) */
    uint32_t           ch_fwd;
    TIM_HandleTypeDef *htim_rev;   /* таймер LPWM (назад)  */
    uint32_t           ch_rev;
    volatile uint32_t *ccr_fwd;
    volatile uint32_t *ccr_rev;
    volatile uint32_t *arr;        /* ARR (оба таймера одинаковый период) */
    float              duty_now;
    float              duty_target;
    PID_t              pid;
} Motor_t;

void motor_init      (Motor_t *m, float dt);
void motor_set_target(Motor_t *m, float duty);
void motor_update    (Motor_t *m, float dt);
void motor_stop      (Motor_t *m);
