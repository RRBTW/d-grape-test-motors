#pragma once
#include "stm32f4xx_hal.h"
#include "robot_config.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void Error_Handler(void);
