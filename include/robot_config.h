#pragma once

/* ============================================================
 *  robot_config.h — D-Grape (BTS7960B x2, STM32F407G Discovery)
 *
 *  Левый мотор:
 *    PA15 → RPWM (вперёд) — TIM2_CH1, AF1
 *    PA7  → LPWM (назад)  — TIM3_CH2, AF2
 *
 *  Правый мотор (физически инвертирован):
 *    PB3  → RPWM (вперёд) — TIM2_CH2, AF1
 *    PB11 → LPWM (назад)  — TIM2_CH4, AF1
 *
 *  EN-пины подтянуты к 5V на плате.
 * ============================================================ */

#include "stm32f4xx_hal.h"

/* ── Левый мотор ─────────────────────────────────────────────*/
#define MOTOR_LEFT_CH_FWD       TIM_CHANNEL_1   /* PA15, TIM2 */
#define MOTOR_LEFT_CCR_FWD      (TIM2->CCR1)
#define MOTOR_LEFT_CH_REV       TIM_CHANNEL_2   /* PA7,  TIM3 */
#define MOTOR_LEFT_CCR_REV      (TIM3->CCR2)

/* ── Правый мотор ────────────────────────────────────────────*/
#define MOTOR_RIGHT_CH_FWD      TIM_CHANNEL_2   /* PB3,  TIM2 */
#define MOTOR_RIGHT_CCR_FWD     (TIM2->CCR2)
#define MOTOR_RIGHT_CH_REV      TIM_CHANNEL_4   /* PB11, TIM2 */
#define MOTOR_RIGHT_CCR_REV     (TIM2->CCR4)

/* ── PID (регулятор плавности скважности) ───────────────────*/
#define PID_KP                  0.3f
#define PID_KI                  0.0f
#define PID_KD                  0.0f
#define PID_OUTPUT_MAX          1.0f
#define PID_OUTPUT_MIN         -1.0f
#define PID_INTEGRAL_MAX        1.0f

/* ── Таймаут команды: мотор останавливается если нет нажатий ─
 *  250 мс — чуть больше интервала автоповтора клавиши (~100 мс),
 *  чуть меньше начальной задержки OS (~300–500 мс).            */
#define CMD_TIMEOUT_MS          250U
