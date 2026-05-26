#ifndef INC_PID_H_
#define INC_PID_H_

/*
 * PID controller state and configuration.
 *
 * kp, ki, and kd are the proportional, integral, and derivative gains.
 * integral_max limits the accumulated error_integral value.
 * pid_max limits the final PID output value.
 *
 * error_integral stores the accumulated error over time.
 * error_previous stores the error from the previous valid update.
 * output stores the latest saturated PID output.
 */
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

/*
 * Initializes a PID controller instance.
 *
 * If the PID pointer is null, the function returns without changing any
 * state. Negative integral_max or pid_max values are converted to their
 * absolute values before being stored.
 */
void pid_init(PID_t *pid,
        	float kp,
			float ki,
			float kd,
			float integral_max,
			float pid_max);

/*
 * Resets the PID controller memory.
 *
 * This clears the accumulated error, the previous error, and the latest
 * output value. The PID gains and output limits are not changed.
 */
void pid_reset(PID_t *pid);

/*
 * Computes one PID controller update.
 *
 * setpoint is the target value.
 * measurement is the current feedback value.
 * dt is the update interval in seconds.
 *
 * If the PID pointer is null, the function returns without changing any
 * state. If dt is less than or equal to zero, the output is cleared and
 * the update is skipped.
 *
 * The integral term is limited by integral_max, and the final output is
 * limited by pid_max.
 */
void pid_compute(PID_t *pid,
                  float setpoint,
                  float measurement,
                  float dt);

#endif /* INC_PID_H_ */
