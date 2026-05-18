#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * 将 PWM 脉宽 us 转换为 TIM2 CCR 值。
 *
 * 输入：
 *   pulse_us：PWM 脉宽，单位 us，典型范围 1000~2000
 *
 * 输出：
 *   TIM2 CCR 比较值
 */
uint32_t PWM_Output_UsToCcr(int16_t pulse_us);

/*
 * 设置电机 PWM 输出脉宽。
 *
 * 功能：
 *   1. 对 pulse_us 做 1000~2000 us 限幅；
 *   2. 转换为 TIM2 CCR；
 *   3. 更新 debug_pwm_us / debug_pwm_ccr；
 *   4. 写入 TIM2 CH2 输出 PWM。
 */
void PWM_Output_Set_US(int16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif
