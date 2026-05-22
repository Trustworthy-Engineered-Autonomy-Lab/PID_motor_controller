#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

#include <stdint.h>
#include "main.h"

/* ============================================================
 * 1. System clock
 * ============================================================
 * 系统定时器时钟频率，单位 Hz。
 *
 * 当前 STM32F103 工程中，定时器时钟按 72 MHz 计算。
 * 如果以后修改系统时钟或定时器时钟来源，需要同步修改这里。
 */
#define CLK_FREQ                    72000000UL


/* ============================================================
 * 2. Hardware timer mapping
 * ============================================================
 * 这里集中定义本项目使用的硬件定时器。
 *
 * htim1 / htim2 / htim3 的实体变量由 CubeMX 在 main.c 中生成。
 * 这里用 extern 声明，是为了让 user.c、motor_control.c 等模块
 * 可以通过下面的配置宏访问对应定时器。
 *
 * 当前分配：
 *   TIM1 -> 控制周期定时器，触发 PID / PWM 周期刷新
 *   TIM2 -> PWM 输出定时器，用于控制 ESC
 *   TIM3 -> Hall 传感器输入捕获 / 超时检测
 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define CONTROL_TIMER_HANDLE        htim1
#define MOTOR_PWM_TIMER_HANDLE      htim2
#define HALL_TIMER_HANDLE           htim3

#define MOTOR_PWM_CHANNEL           TIM_CHANNEL_2
#define HALL_CAPTURE_CHANNEL        TIM_CHANNEL_1

/* ============================================================
 * 3. Timer period calculation
 * ============================================================
 * 定时器周期计算，单位 ms。
 *
 * 计算公式：
 *   period_ms = (PSC + 1) * (ARR + 1) / timer_clock * 1000
 *
 * TIMx_PSC 和 TIMx_CTR_PER 应该来自 user.h / main.h / 其他配置头文件。
 * 如果 CubeMX 中修改了 Prescaler 或 Counter Period，
 * 这些宏需要保持一致，否则 TIM_PER_CHECK() 会报错。
 */
#define TIM1_PER_MS                 \
    (1.0f / CLK_FREQ * (TIM1_PSC + 1) * (TIM1_CTR_PER + 1) * 1000.0f)

#define TIM2_PER_MS                 \
    (1.0f / CLK_FREQ * (TIM2_PSC + 1) * (TIM2_CTR_PER + 1) * 1000.0f)

#define TIM3_PER_MS                 \
    (1.0f / CLK_FREQ * (TIM3_PSC + 1) * (TIM3_CTR_PER + 1) * 1000.0f)


/* ============================================================
 * 4. Expected timer periods
 * ============================================================
 * 期望定时器周期，单位 ms。
 *
 * 用于 TIM_PER_CHECK() 检查 CubeMX 定时器配置是否正确。
 *
 * 当前设计：
 *   TIM1 = 10 ms   -> 控制循环周期，约 100 Hz
 *   TIM2 = 20 ms   -> ESC PWM 周期，约 50 Hz
 *   TIM3 = 200 ms  -> Hall 低速超时检测周期
 */
#define EXPECTED_TIM1_PER_MS        10.0f
#define EXPECTED_TIM2_PER_MS        20.0f
#define EXPECTED_TIM3_PER_MS        200.0f


/* ============================================================
 * 5. ESC PWM pulse width config
 * ============================================================
 * ESC 控制脉宽范围。
 *
 * 注意：
 *   PWM_MIN_PULSEWIDTH / PWM_ZERO_PULSEWIDTH / PWM_MAX_PULSEWIDTH
 *   的单位是 ms，用于 PID 输出计算和 CCR 换算。
 *
 *   PWM_US_MIN / PWM_US_NEUTRAL / PWM_US_MAX
 *   的单位是 us，用于上位机寄存器输入和更直观的 PWM 输出接口。
 *
 * 当前假设：
 *   1000 us = 最小油门 / 刹车端
 *   1500 us = 中位 / 停止
 *   2000 us = 最大油门
 */
#define PWM_MIN_PULSEWIDTH          1.0f
#define PWM_ZERO_PULSEWIDTH         1.5f
#define PWM_MAX_PULSEWIDTH          2.0f

#define PWM_US_MIN                  1000
#define PWM_US_NEUTRAL              1500
#define MOTOR_MIN_START_US          1550
#define PWM_US_MAX                  2000


/* ============================================================
 * 6. PWM frequency and CCR conversion
 * ============================================================
 * TIM2 用于输出 ESC PWM。
 *
 * PWM_FREQUENCY:
 *   根据 TIM2 周期计算 PWM 频率，单位 Hz。
 *   如果 TIM2_PER_MS = 20 ms，则 PWM_FREQUENCY = 50 Hz。
 *
 * CCR 换算逻辑：
 *   pulse_width_ms -> duty_cycle -> CCR
 *
 * 注意：
 *   PWM_US_TO_CCR(us) 是当前推荐使用的接口。
 *   motor_control.c 中的 Motor_Control_Set_PWM_US() 会调用它。
 */
#define PWM_FREQUENCY               \
    (1.0f / TIM2_PER_MS * 1000.0f)

#define PWM_PULSEWIDTH_TO_DUTYCYCLE(pulse_ms) \
    ((pulse_ms) / 1000.0f * PWM_FREQUENCY)

#define PWM_DUTYCYCLE_TO_CCR(duty)  \
    ((uint32_t)((duty) * TIM2_CTR_PER))

#define PWM_PULSEWIDTH_TO_CCR(pulse_ms) \
    PWM_DUTYCYCLE_TO_CCR(PWM_PULSEWIDTH_TO_DUTYCYCLE(pulse_ms))

#define PWM_US_TO_MS(us)            \
    ((float)(us) / 1000.0f)

#define PWM_US_CLAMP(us)            \
    (((us) < PWM_US_MIN) ? PWM_US_MIN : (((us) > PWM_US_MAX) ? PWM_US_MAX : (us)))

#define PWM_US_TO_CCR(us)           \
    ((uint32_t)PWM_PULSEWIDTH_TO_CCR(PWM_US_TO_MS(PWM_US_CLAMP(us))))


/* ============================================================
 * 7. Common PWM CCR values
 * ============================================================
 * 常用 PWM CCR 值。
 *
 * 这些宏主要用于调试或旧代码兼容。
 * 新代码优先使用 PWM_US_TO_CCR(us)。
 */
#define PWM_CCR_MIN                 \
    PWM_PULSEWIDTH_TO_CCR(PWM_MIN_PULSEWIDTH)

#define PWM_CCR_DEFAULT             \
    PWM_PULSEWIDTH_TO_CCR(PWM_ZERO_PULSEWIDTH)

#define PWM_CCR_MAX                 \
    PWM_PULSEWIDTH_TO_CCR(PWM_MAX_PULSEWIDTH)


/* ============================================================
 * 8. Hall speed measurement
 * ============================================================
 * Hall 测速参数。
 *
 * HALL_EDGES_PER_REV:
 *   电机每转一圈对应的 Hall 边沿数量。
 *   当前按 12 个边沿 / 转计算。
 *
 * HALL_CAPTURE_TO_RPM(capture_value):
 *   根据 TIM3 输入捕获计数值计算 RPM。
 *
 * 计算逻辑：
 *   单个边沿间隔时间 = capture_value * (TIM3_PSC + 1) / CLK_FREQ
 *   边沿频率 = 1 / 单个边沿间隔时间
 *   转速 RPM = 边沿频率 * 60 / 每圈边沿数
 */
#define HALL_EDGES_PER_REV          12.0f

#define HALL_CAPTURE_TO_RPM(capture_value) \
    ((CLK_FREQ * 60.0f) / ((float)(capture_value) * (TIM3_PSC + 1) * HALL_EDGES_PER_REV))

/*
 * 最低可测 RPM。
 *
 * 当 TIM3 计数达到最大周期仍未捕获到新 Hall 边沿时，
 * 可以认为电机速度低于该值，或者已经停止。
 */
#define MIN_MOTOR_RPM               \
    HALL_CAPTURE_TO_RPM(TIM3_CTR_PER)


/* ============================================================
 * 9. RPM low-pass filter
 * ============================================================
 * 一阶低通滤波系数。
 *
 * alpha 越小，滤波越平滑，但响应越慢；
 * alpha 越大，响应越快，但保留更多波动。
 *
 * 当前滤波在 TIM1 控制周期中运行，
 * 因此实际滤波效果与 TIM1 周期有关。
 */
#define RPM_FILTER_ALPHA            0.05f


#endif /* INC_APP_CONFIG_H_ */
