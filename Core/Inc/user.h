/*
 * user.h
 *
 *  Created on: Feb 2, 2026
 *      Author: Tyler Ruble
 */

#ifndef INC_USER_H_
#define INC_USER_H_

/* Includes */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "pid.h"

/* End Includes */

/* Macros */

/* Typical RC ESC's (like the one we are currently using) control the motor based off of the pulse
 * width in the received PWM signal, with the default/zero value being 1.5ms, and a standard range
 * of 1-2ms
 */
#define PWM_MIN_PULSEWIDTH		1.0f
#define PWM_ZERO_PULSEWIDTH		1.5f
#define PWM_MAX_PULSEWIDTH		2.0f

#define PWM_FREQUENCY		50.0f	// Frequency of PWM output timer, TIM2
#define PWM_TIM_ARR			14400	// Auto Reload Register Value (the Maximum value our timer will count to)

#define PWM_CCR_MIN			(PWM_MIN_PULSEWIDTH * PWM_FREQUENCY) * PWM_TIM_ARR - 1
#define PWM_CCR_DEFAULT		(PWM_ZERO_PULSEWIDTH * PWM_FREQUENCY) * PWM_TIM_ARR - 1
#define PWM_CCR_MAX      	(PWM_MAX_PULSEWIDTH * PWM_FREQUENCY) * PWM_TIM_ARR - 1


#define PWM_PULSEWIDTH_TO_DUTYCYCLE(p) ((p)/PWM_FREQUENCY)
#define PWM_DUTYCYCLE_TO_CCR(d) ((d) * PWM_TIM_ARR - 1)
#define PWM_PULSEWIDTH_TO_CCR(p) DUTYCYCLE_TO_CCR(PULSEWIDTH_TO_DUTYCYCLE(p))

/* End Macros */



/* Function Declarations */
void User_Init(void);
void loop(void);


/* End Function Declarations */



#endif /* INC_USER_H_ */
