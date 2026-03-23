#pragma once

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_max;
    float output_min;
    float integral_max;
    float integral;
    float prev_measured;
    float dt;
    float setpoint;
    float measured;
    float output;
    uint8_t enabled;
} PID_t;

void pid_init(PID_t *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_max, float dt);
void pid_update(PID_t *pid);
void pid_reset(PID_t *pid);

static inline void pid_enable(PID_t *pid)  { pid->enabled = 1; }
static inline void pid_disable(PID_t *pid) { pid->enabled = 0; pid_reset(pid); }
