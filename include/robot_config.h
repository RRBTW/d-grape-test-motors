#pragma once

/* ============================================================
 *  robot_config.h — Конфигурация D-Grape (BTS7960B, без энкодеров)
 * ============================================================ */

#include "stm32f4xx_hal.h"

/* ── Моторы: TIM2, 20 кГц ───────────────────────────────────
 *  Левый:   RPWM = PA15 (CH1), LPWM = PB10 (CH3)
 *  Правый:  RPWM = PB3  (CH2), LPWM = PB11 (CH4)
 * ─────────────────────────────────────────────────────────── */
#define MOTOR_TIM                   TIM2

#define MOTOR_LEFT_CH_FWD           TIM_CHANNEL_1
#define MOTOR_LEFT_CCR_FWD          (TIM2->CCR1)
#define MOTOR_LEFT_CH_REV           TIM_CHANNEL_3
#define MOTOR_LEFT_CCR_REV          (TIM2->CCR3)

#define MOTOR_RIGHT_CH_FWD          TIM_CHANNEL_2
#define MOTOR_RIGHT_CCR_FWD         (TIM2->CCR2)
#define MOTOR_RIGHT_CH_REV          TIM_CHANNEL_4
#define MOTOR_RIGHT_CCR_REV         (TIM2->CCR4)

/* ── EN-пины BTS7960B (PD0-PD3, держать HIGH) ───────────────
 *  Левый:   R_EN = PD0, L_EN = PD1
 *  Правый:  R_EN = PD2, L_EN = PD3
 * ─────────────────────────────────────────────────────────── */
#define MOTOR_LEFT_EN_PORT          GPIOD
#define MOTOR_LEFT_EN_R_PIN         GPIO_PIN_0
#define MOTOR_LEFT_EN_L_PIN         GPIO_PIN_1

#define MOTOR_RIGHT_EN_PORT         GPIOD
#define MOTOR_RIGHT_EN_R_PIN        GPIO_PIN_2
#define MOTOR_RIGHT_EN_L_PIN        GPIO_PIN_3

/* ── PID (регулятор плавности скважности) ───────────────────
 *  setpoint = duty_target, measured = duty_now
 *  Kp=0.3, Ki=0, Kd=0 → экспоненциальный подход, τ≈30 мс @100 Гц
 * ─────────────────────────────────────────────────────────── */
#define PID_KP                      0.3f
#define PID_KI                      0.0f
#define PID_KD                      0.0f
#define PID_OUTPUT_MAX              1.0f
#define PID_OUTPUT_MIN             -1.0f
#define PID_INTEGRAL_MAX            1.0f

/* ── Таймаут команды ────────────────────────────────────────*/
#define CMD_TIMEOUT_MS              500U

/* ── LED ────────────────────────────────────────────────────*/
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
