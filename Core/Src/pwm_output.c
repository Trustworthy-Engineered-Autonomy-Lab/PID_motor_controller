#include "pwm_output.h"
#include "user.h"

/*
 * htim2 由 CubeMX 在 tim.c / main.c 中生成。
 * 这里 extern 引用，用于写 TIM2 CH2 的 PWM CCR。
 */
extern TIM_HandleTypeDef htim2;

/*
 * 这两个调试变量目前仍然定义在 user.c 中。
 * 本步骤暂时不移动变量，只在这里 extern 引用。
 *
 * 后续如果继续分装，可以再统一移动到 debug_vars.c。
 */
extern volatile int16_t debug_pwm_us;
extern volatile uint32_t debug_pwm_ccr;

/*
 * 将 PWM 脉宽 us 转换为 TIM2 CCR。
 */
uint32_t PWM_Output_UsToCcr(int16_t pulse_us)
{
    pulse_us = PWM_US_CLAMP(pulse_us);

    return PWM_US_TO_CCR(pulse_us);
}

/*
 * 设置 PWM 输出。
 */
void PWM_Output_Set_US(int16_t pulse_us)
{
    pulse_us = PWM_US_CLAMP(pulse_us);

    uint32_t pwm_ccr = PWM_Output_UsToCcr(pulse_us);

    debug_pwm_us = pulse_us;
    debug_pwm_ccr = pwm_ccr;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_ccr);
}


