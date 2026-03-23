#pragma once

#include "stm32f4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t            count_prev;
    float              speed_mps;
    float              distance_m;
} Encoder_t;

void encoder_init(Encoder_t *enc);
void encoder_update(Encoder_t *enc, float dt);
void encoder_reset(Encoder_t *enc);
