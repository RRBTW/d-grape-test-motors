#pragma once

#include "stm32f4xx_hal.h"
#include "pid.h"
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           ch_fwd;       /* RPWM — канал «вперёд» */
    uint32_t           ch_rev;       /* LPWM — канал «назад»  */
    volatile uint32_t *ccr_fwd;      /* CCR для RPWM          */
    volatile uint32_t *ccr_rev;      /* CCR для LPWM          */
    volatile uint32_t *arr;          /* ARR (период таймера)  */
    GPIO_TypeDef      *en_port;      /* порт EN-пинов         */
    uint16_t           en_r_pin;     /* R_EN пин              */
    uint16_t           en_l_pin;     /* L_EN пин              */
    float              duty_now;     /* текущая скважность [-1..1] */
    float              duty_target;  /* целевая скважность [-1..1] */
    PID_t              pid;          /* регулятор плавности        */
} Motor_t;

void motor_init      (Motor_t *m, float dt);
void motor_set_target(Motor_t *m, float duty);
void motor_update    (Motor_t *m, float dt);
void motor_stop      (Motor_t *m);
