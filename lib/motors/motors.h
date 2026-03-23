#pragma once

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "encoders.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    volatile uint32_t *ccr;
    volatile uint32_t *arr;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    PID_t              pid;
    Encoder_t         *enc;
} Motor_t;

void motor_init(Motor_t *m, Encoder_t *enc, float dt);
void motor_velocity_update(Motor_t *m, float target_mps, float dt);
void motor_set_duty(Motor_t *m, float duty);
void motor_stop(Motor_t *m);
