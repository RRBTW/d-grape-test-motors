#include "motors.h"
#include "robot_config.h"

void motor_init(Motor_t *m, Encoder_t *enc, float dt)
{
    m->enc = enc;
    pid_init(&m->pid,
             PID_KP, PID_KI, PID_KD,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX,
             PID_INTEGRAL_MAX,
             dt);
    HAL_TIM_PWM_Start(m->htim, m->channel);
    motor_stop(m);
}

void motor_velocity_update(Motor_t *m, float target_mps, float dt)
{
    if (m->enc == NULL) { motor_set_duty(m, 0.0f); return; }
    m->pid.dt       = dt;
    m->pid.setpoint = target_mps;
    m->pid.measured = m->enc->speed_mps;
    pid_update(&m->pid);
    motor_set_duty(m, m->pid.output);
}

void motor_set_duty(Motor_t *m, float duty)
{
    if      (duty >  1.0f) duty =  1.0f;
    else if (duty < -1.0f) duty = -1.0f;

    uint32_t period = *m->arr;
    if (duty >= 0.0f) {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_RESET);
        *m->ccr = (uint32_t)(duty * (float)period);
    } else {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_SET);
        *m->ccr = (uint32_t)(-duty * (float)period);
    }
}

void motor_stop(Motor_t *m)
{
    *m->ccr = 0;
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_RESET);
    pid_reset(&m->pid);
}
