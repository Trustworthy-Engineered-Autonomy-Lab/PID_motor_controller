/*
 * pid.h
 *
 *  Created on: Feb 6, 2026
 *      Author: Eldrulf
 */

#ifndef INC_PID_H_
#define INC_PID_H_

// PID structure
typedef struct{
	float kp;
	float ki;
	float kd;

	float integral_max;
	float pid_max;

	float error_integral;
	float error_previous;

	float output;

} PID_t;

// PID Init function
void PID_Init(PID_t *pid,
        	float kp,
			float ki,
			float kd,
			float integral_max,
			float pid_max);

// Resets integral and derivative memory
void PID_Reset(PID_t *pid);

// Computes PID output
void PID_Compute(PID_t *pid,
                  float setpoint, // Desired value
                  float measurement, // Current value
                  float dt); // time step in seconds

#endif /* INC_PID_H_ */
