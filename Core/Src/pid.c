#include "pid.h"

/*
 * Initializes a PID controller instance.
 *
 * The function stores the PID gains, clears the controller memory, and
 * stores the absolute values of the integral and output limits.
 */
void pid_init(PID_t *pid,
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

/*
 * Resets the PID controller memory.
 *
 * The PID gains and configured limits are left unchanged.
 */
void pid_reset(PID_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->error_previous = 0.0f;
    pid->error_integral = 0.0f;
    pid->output = 0.0f;
}

/*
 * Computes one PID controller update.
 *
 * If dt is less than or equal to zero, the output is cleared and the
 * update is skipped.
 *
 * The accumulated error is limited by integral_max before the integral
 * term is computed. The final PID output is limited by pid_max.
 */
void pid_compute(PID_t *pid,
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

    /*
     * Proportional term.
     */
    float p = pid->kp * error;

    /*
     * Integral term with accumulated-error limiting.
     */
    pid->error_integral += error * dt;

    if (pid->error_integral > pid->integral_max) {
        pid->error_integral = pid->integral_max;
    } else if (pid->error_integral < -pid->integral_max) {
        pid->error_integral = -pid->integral_max;
    }

    float i = pid->ki * pid->error_integral;

    /*
     * Derivative term based on the change in error.
     */
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
