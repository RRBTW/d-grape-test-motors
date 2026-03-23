#include "motors.h"
#include "robot_config.h"

/* Применяет скважность напрямую к CCR (без PID) */
static void motor_apply_duty(Motor_t *m, float duty)
{
    if      (duty >  1.0f) duty =  1.0f;
    else if (duty < -1.0f) duty = -1.0f;

    uint32_t period = *m->arr;
    if (duty >= 0.0f) {
        *m->ccr_fwd = (uint32_t)(duty * (float)period);
        *m->ccr_rev = 0U;
    } else {
        *m->ccr_fwd = 0U;
        *m->ccr_rev = (uint32_t)(-duty * (float)period);
    }
}

void motor_init(Motor_t *m, float dt)
{
    pid_init(&m->pid,
             PID_KP, PID_KI, PID_KD,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX,
             PID_INTEGRAL_MAX,
             dt);
    pid_enable(&m->pid);

    m->duty_now    = 0.0f;
    m->duty_target = 0.0f;

    HAL_TIM_PWM_Start(m->htim, m->ch_fwd);
    HAL_TIM_PWM_Start(m->htim, m->ch_rev);

    /* EN-пины HIGH — разрешить драйвер */
    HAL_GPIO_WritePin(m->en_port, m->en_r_pin | m->en_l_pin, GPIO_PIN_SET);

    motor_apply_duty(m, 0.0f);
}

void motor_set_target(Motor_t *m, float duty)
{
    if      (duty >  1.0f) duty =  1.0f;
    else if (duty < -1.0f) duty = -1.0f;
    m->duty_target = duty;
}

void motor_update(Motor_t *m, float dt)
{
    m->pid.dt       = dt;
    m->pid.setpoint = m->duty_target;
    m->pid.measured = m->duty_now;
    pid_update(&m->pid);
    m->duty_now = m->pid.output;
    motor_apply_duty(m, m->duty_now);
}

void motor_stop(Motor_t *m)
{
    m->duty_target = 0.0f;
    m->duty_now    = 0.0f;
    *m->ccr_fwd    = 0U;
    *m->ccr_rev    = 0U;
    pid_reset(&m->pid);
    pid_enable(&m->pid);
}
