#ifndef INC_PWM_OUTPUT_H_
#define INC_PWM_OUTPUT_H_

#include <stdint.h>

extern volatile int16_t debug_pwm_us;
extern volatile uint32_t debug_pwm_ccr;

uint32_t PWM_Output_UsToCcr(int16_t pulse_us);
void PWM_Output_Set_US(int16_t pulse_us);

#endif
