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
			  float integral_max,
			  float pid_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->error_integral = 0.0f;
    pid->error_previous = 0.0f;

    pid->integral_max = integral_max;
    pid->pid_max = pid_max;
}


// Resets integral and derivative memory
void PID_Reset(PID_t *pid)
{
    pid->error_previous = 0.0f;
    pid->error_integral = 0.0f;
}


// Computes PID output
void PID_Compute(PID_t *pid,
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

    pid->output = p + i + d;

    if(pid->output >= pid->pid_max){
    	pid->output = pid->pid_max;
    }
    if(pid->output <= (-1)*pid->pid_max)
    {
    	pid->output = (-1)*pid->pid_max;
    }


    pid->error_previous = error;

}
