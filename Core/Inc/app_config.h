#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

#include <stdint.h>
#include "main.h"

/* 系统时钟 */
#define CLK_FREQ 72000000

/* 定时器周期，单位 ms */
#define TIM1_PER_MS \
    (1.0f / CLK_FREQ * (TIM1_PSC + 1) * (TIM1_CTR_PER + 1) * 1000.0f)

#define TIM2_PER_MS \
    (1.0f / CLK_FREQ * (TIM2_PSC + 1) * (TIM2_CTR_PER + 1) * 1000.0f)

#define TIM3_PER_MS \
    (1.0f / CLK_FREQ * (TIM3_PSC + 1) * (TIM3_CTR_PER + 1) * 1000.0f)

/* 期望定时器周期 */
#define EXPECTED_TIM1_PER_MS 10
#define EXPECTED_TIM2_PER_MS 20
#define EXPECTED_TIM3_PER_MS 200

/* ESC PWM 脉宽，单位 ms */
#define PWM_MIN_PULSEWIDTH 1.0f
#define PWM_ZERO_PULSEWIDTH 1.5f
#define PWM_MAX_PULSEWIDTH 2.0f

/* ESC PWM 脉宽，单位 us */
#define PWM_US_MIN 1000
#define PWM_US_NEUTRAL 1500
#define MOTOR_MIN_START_US 1550
#define PWM_US_MAX 2000

#define PWM_FREQUENCY \
    (1.0f / TIM2_PER_MS * 1000.0f)

#define PWM_CCR_MIN \
    ((uint32_t)((PWM_MIN_PULSEWIDTH / 1000.0f * PWM_FREQUENCY) * (TIM2_CTR_PER + 1) - 1))

#define PWM_CCR_DEFAULT \
    ((uint32_t)((PWM_ZERO_PULSEWIDTH / 1000.0f * PWM_FREQUENCY) * (TIM2_CTR_PER + 1) - 1))

#define PWM_CCR_MAX \
    ((uint32_t)((PWM_MAX_PULSEWIDTH / 1000.0f * PWM_FREQUENCY) * (TIM2_CTR_PER + 1) - 1))

#define PWM_PULSEWIDTH_TO_DUTYCYCLE(p) \
    ((p) / 1000.0f * PWM_FREQUENCY)

#define PWM_DUTYCYCLE_TO_CCR(d) \
    ((uint32_t)((d) * TIM2_CTR_PER))

#define PWM_PULSEWIDTH_TO_CCR(p) \
    PWM_DUTYCYCLE_TO_CCR(PWM_PULSEWIDTH_TO_DUTYCYCLE(p))

#define PWM_US_TO_MS(us) \
    ((float)(us) / 1000.0f)

#define PWM_US_CLAMP(us) \
    (((us) < PWM_US_MIN) ? PWM_US_MIN : (((us) > PWM_US_MAX) ? PWM_US_MAX : (us)))

#define PWM_US_TO_CCR(us) \
    ((uint32_t)PWM_PULSEWIDTH_TO_CCR(PWM_US_TO_MS(PWM_US_CLAMP(us))))

/* Hall 测速 */
#define HALL_EDGES_PER_REV 12.0f

/* RPM low-pass filter */
#define RPM_FILTER_ALPHA 0.05f

/*
 * Motor PWM uses TIM2 Channel 2.
 * The actual TIM handle is defined in main.c.
 * motor_control.c must declare:
 * extern TIM_HandleTypeDef htim2;
 */
#define MOTOR_PWM_TIMER_HANDLE htim2
#define MOTOR_PWM_CHANNEL      TIM_CHANNEL_2
extern TIM_HandleTypeDef htim2;

#define HALL_CAPTURE_TO_RPM(capture_value) \
    ((CLK_FREQ * 60.0f) / ((float)(capture_value) * (TIM3_PSC + 1) * HALL_EDGES_PER_REV))

#define MIN_MOTOR_RPM \
    HALL_CAPTURE_TO_RPM(TIM3_CTR_PER)

#endif /* INC_APP_CONFIG_H_ */

