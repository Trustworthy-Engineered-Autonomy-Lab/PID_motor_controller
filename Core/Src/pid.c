/*
 * pid.c
 *
 *  Created on: Feb 6, 2026
 *      Author: Eldrulf
 */

#include "pid.h"

// PID Init function
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
			  float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->error_integral = 0.0f;
    pid->error_previous = 0.0f;

    pid->integral_max = integral_max;
}


// Resets integral and derivative memory
void PID_Reset(PID_t *pid)
{
    pid->error_previous = 0.0f;
    pid->error_integral = 0.0f;
}


// Computes PID output
float PID_Compute(PID_t *pid,
                  float setpoint,
                  float measurement,
                  float dt)
{
    float error = setpoint - measurement;

    /* Proportional */
    float p = pid->kp * error;

    /* Integral */

    pid->error_integral += error * dt;

	// Anti-windup for integral path
	if(pid->error_integral > pid->integral_max) {
		pid->error_integral = pid->integral_max;
	} else if (pid->error_integral < (-1)*(pid->integral_max)) {
		pid->error_integral = (-1)*(pid->integral_max);
	}

    float i = pid->ki * pid->error_integral;

    /* Derivative */
    float derivative = (error - pid->error_previous) / dt;
    float d = pid->kd * derivative;

    float output = p + i + d;


    pid->error_previous = error;

    return output;
}
