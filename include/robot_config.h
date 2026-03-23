#pragma once

/* ============================================================
 *  robot_config.h — Конфигурация D-Grape (test version, без FreeRTOS)
 * ============================================================ */

#include "stm32f4xx_hal.h"

#define ROBOT_WHEEL_RADIUS          0.0877f
#define ROBOT_TRACK_WIDTH           0.526f

#define ENCODER_PPR                 4096U
#define ENCODER_TIMER_PERIOD        0xFFFFU
#define ENCODER_LEFT_TIM            TIM3
#define ENCODER_RIGHT_TIM           TIM4
#define ENCODER_SKIS_TIM            TIM5

#define MOTOR_LEFT_TIM              TIM2
#define MOTOR_LEFT_CHANNEL          TIM_CHANNEL_1
#define MOTOR_LEFT_CCR              (TIM2->CCR1)
#define MOTOR_RIGHT_TIM             TIM2
#define MOTOR_RIGHT_CHANNEL         TIM_CHANNEL_2
#define MOTOR_RIGHT_CCR             (TIM2->CCR2)
#define MOTOR_SKIS_TIM              TIM1
#define MOTOR_SKIS_CHANNEL          TIM_CHANNEL_1
#define MOTOR_SKIS_CCR              (TIM1->CCR1)

#define MOTOR_LEFT_DIR_PORT         GPIOD
#define MOTOR_LEFT_DIR_PIN          GPIO_PIN_0
#define MOTOR_RIGHT_DIR_PORT        GPIOD
#define MOTOR_RIGHT_DIR_PIN         GPIO_PIN_1
#define MOTOR_SKIS_DIR_PORT         GPIOD
#define MOTOR_SKIS_DIR_PIN          GPIO_PIN_2

#define PID_KP                      1.5f
#define PID_KI                      0.1f
#define PID_KD                      0.05f
#define PID_OUTPUT_MAX              1.0f
#define PID_OUTPUT_MIN             -1.0f
#define PID_INTEGRAL_MAX            5.0f

#define IMU_I2C_ADDR                (0x68U << 1)
#define IMU_I2C_TIMEOUT_MS          1U
#define IMU_ACCEL_FS_G              2
#define IMU_GYRO_FS_DPS             250
#define IMU_DLPF_CFG                3
#define IMU_SMPLRT_DIV              1U

#define CMD_TIMEOUT_MS              500U

#define LED_IMU_PORT        GPIOD
#define LED_IMU_PIN         GPIO_PIN_12
#define LED_MOTORS_PORT     GPIOD
#define LED_MOTORS_PIN      GPIO_PIN_13
#define LED_HEARTBEAT_PORT  GPIOD
#define LED_HEARTBEAT_PIN   GPIO_PIN_14
#define LED_COMM_PORT       GPIOD
#define LED_COMM_PIN        GPIO_PIN_15
#define LED_ERROR_PORT      GPIOE
#define LED_ERROR_PIN       GPIO_PIN_1
