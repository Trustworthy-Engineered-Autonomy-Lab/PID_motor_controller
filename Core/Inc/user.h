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
#define PWM_MAX_DUTY        50      // Maximum PWM compare value (adjust based on your timer config)
#define MAX_PULSEWIDTH		2.0f	// 2ms = Max pulsewidth servo will handle
#define TOTAL_PERIOD		50.0f	// 1 / 20 Hz = 50ms
#define MAX_CCR				36000	// Max CCR = Auto Reload Register Value (the Maximum value our timer will count to)
#define CCR_MAX_VALUE       (MAX_PULSEWIDTH/TOTAL_PERIOD)*MAX_CCR - 1 // 1440 is the corresponding 4% CCR value

#define PULSE_WIDTH_TO_DUTYCYCLE(p) ((p)/50.0f)
#define DUTYCYCLE_TO_CCR(d) ((d)*36000-1)
#define PULSE_WIDTH_TO_CCR(p) DUTYCYCLE_TO_CCR(PULSE_WIDTH_TO_DUTYCYCLE(p))

/* End Macros */



/* Function Declarations */
void User_Init(void);
void user_loop(void);


/* End Function Declarations */



#endif /* INC_USER_H_ */
