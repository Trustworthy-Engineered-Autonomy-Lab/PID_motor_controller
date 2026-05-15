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
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_bus.h"
#include "main.h"
#include "pid.h"
#include <math.h>

/* End Includes */

/* Macros */

/* Typical RC ESC's (like the one we are currently using) control the motor based off of the pulse
 * width in the received PWM signal, with the default/zero value being 1.5ms, and a standard range
 * of 1-2ms
 */

// FIXME: Update macros for fixed point arithmetic
// Timer periods, remember to change if timer changes are made:
#define CLK_FREQ	72000000  // Frequency of internal clock used for timers

// Periods in ms
#define TIM1_PER_MS	(1.0f / CLK_FREQ * (TIM1_PSC + 1) * (TIM1_CTR_PER + 1) * 1000) // 15 ms
#define TIM2_PER_MS	(1.0f / CLK_FREQ * (TIM2_PSC + 1) * (TIM2_CTR_PER + 1) * 1000) // 20 ms
#define TIM3_PER_MS	(1.0f / CLK_FREQ * (TIM3_PSC + 1) * (TIM3_CTR_PER + 1) * 1000) // 200 ms

#define EXPECTED_TIM1_PER_MS	10 // For PID Update Frequency of 60Hz
#define EXPECTED_TIM2_PER_MS	20 // For PWM Frequency of 50Hz (Standard for ESCs)
#define EXPECTED_TIM3_PER_MS	200 // Detects a minimum RPM of 50

// Values determined by ESC convention
#define PWM_MIN_PULSEWIDTH		1.0f // in ms
#define PWM_ZERO_PULSEWIDTH		1.5f // in ms
#define PWM_MAX_PULSEWIDTH		2.0f // in ms

#define PWM_FREQUENCY		(1.0f / TIM2_PER_MS * 1000)	// Frequency of PWM output timer, TIM2

#define PWM_CCR_MIN			(uint32_t)((PWM_MIN_PULSEWIDTH / 1000 * PWM_FREQUENCY) * (TIM2_CTR_PER + 1) - 1) //719
#define PWM_CCR_DEFAULT		(uint32_t)((PWM_ZERO_PULSEWIDTH / 1000 * PWM_FREQUENCY) * (TIM2_CTR_PER + 1) - 1) // 1079
#define PWM_CCR_MAX      	(uint32_t)((PWM_MAX_PULSEWIDTH / 1000 * PWM_FREQUENCY) * (TIM2_CTR_PER + 1) - 1) // 1439


#define PWM_PULSEWIDTH_TO_DUTYCYCLE(p) ((p) / 1000 *PWM_FREQUENCY)
#define PWM_DUTYCYCLE_TO_CCR(d) (uint32_t)((d) * TIM2_CTR_PER)
#define PWM_PULSEWIDTH_TO_CCR(p) PWM_DUTYCYCLE_TO_CCR(PWM_PULSEWIDTH_TO_DUTYCYCLE(p))


#define HALL_EDGES_PER_REV 12.0f

#define HALL_CAPTURE_TO_RPM(capture_value) \
    ((CLK_FREQ * 60.0f) / ((float)(capture_value) * (TIM3_PSC + 1) * HALL_EDGES_PER_REV))

#define MIN_MOTOR_RPM HALL_CAPTURE_TO_RPM(TIM3_CTR_PER)


#define PWM_US_MIN              1000
#define PWM_US_NEUTRAL          1500
#define MOTOR_MIN_START_US      1550
#define PWM_US_MAX              2000

#define PWM_US_TO_MS(us)        ((float)(us) / 1000.0f)

#define PWM_US_CLAMP(us)        \
    (((us) < PWM_US_MIN) ? PWM_US_MIN : (((us) > PWM_US_MAX) ? PWM_US_MAX : (us)))

#define PWM_US_TO_CCR(us)       \
    ((uint32_t)PWM_PULSEWIDTH_TO_CCR(PWM_US_TO_MS(PWM_US_CLAMP(us))))

/* End Macros */



/* Function Declarations */
void User_Init(void);
void User_Loop(void);


extern volatile int16_t debug_pwm_us;
extern volatile uint32_t debug_pwm_ccr;

/* End Function Declarations */



#endif /* INC_USER_H_ */
