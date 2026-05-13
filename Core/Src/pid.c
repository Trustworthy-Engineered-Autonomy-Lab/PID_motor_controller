#include "pid.h"

// PID Init function
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_max,
              float pid_max)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->error_integral = 0.0f;
    pid->error_previous = 0.0f;
    pid->output = 0.0f;

    if (integral_max < 0.0f) {
        integral_max = -integral_max;
    }

    if (pid_max < 0.0f) {
        pid_max = -pid_max;
    }

    pid->integral_max = integral_max;
    pid->pid_max = pid_max;
}


// Resets integral and derivative memory
void PID_Reset(PID_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->error_previous = 0.0f;
    pid->error_integral = 0.0f;
    pid->output = 0.0f;
}


// Computes PID output
void PID_Compute(PID_t *pid,
                 float setpoint,
                 float measurement,
                 float dt)
{
    if (pid == 0) {
        return;
    }

    if (dt <= 0.0f) {
        pid->output = 0.0f;
        return;
    }

    float error = setpoint - measurement;

    /* Proportional */
    float p = pid->kp * error;

    /* Integral */
    pid->error_integral += error * dt;

    if (pid->error_integral > pid->integral_max) {
        pid->error_integral = pid->integral_max;
    } else if (pid->error_integral < -pid->integral_max) {
        pid->error_integral = -pid->integral_max;
    }

    float i = pid->ki * pid->error_integral;

    /* Derivative */
    float derivative = (error - pid->error_previous) / dt;
    float d = pid->kd * derivative;

    pid->output = p + i + d;

    if (pid->output > pid->pid_max) {
        pid->output = pid->pid_max;
    } else if (pid->output < -pid->pid_max) {
        pid->output = -pid->pid_max;
    }

    pid->error_previous = error;
}
