#include "rpm_filter.h"

/*
 * 霍尔测速得到的原始瞬时 RPM。
 *
 * 每次 TIM3 输入捕获时，根据单次霍尔边沿间隔计算。
 * 该值响应最快，但低速时容易波动。
 */
volatile float motor_rpm_raw = 0.0f;

/*
 * 滤波后的 RPM。
 *
 * 该值由 motor_rpm_raw 经过一阶低通滤波得到，
 * 更适合在 STM32CubeMonitor 中观察，也更适合作为 PID 反馈值。
 */
volatile float motor_rpm_filtered = 0.0f;

/*
 * 兼容旧变量名。
 *
 * 当前 motor_rpm 同步为 motor_rpm_filtered。
 */
volatile float motor_rpm = 0.0f;

/*
 * 最新一次霍尔捕获得到的原始 RPM。
 *
 * 霍尔中断只更新这个变量；
 * 滤波器在 TIM1 控制周期中运行，
 * 使滤波采样周期与 PID 采样周期一致。
 */
volatile float latest_raw_rpm = 0.0f;

/*
 * 调试用：观察滤波器是否按 TIM1 周期运行。
 * 如果 TIM1 = 10 ms，该变量每秒应增加约 100。
 */
volatile uint32_t debug_filter_update_count = 0;

/*
 * RPM 一阶低通滤波函数。
 *
 * 输入：
 *   raw_rpm：由单次霍尔捕获计算得到的原始瞬时 RPM。
 *
 * 输出：
 *   motor_rpm_filtered：滤波后的 RPM。
 *   motor_rpm：同步为滤波后的 RPM，供旧逻辑和 PID 使用。
 *
 * 滤波公式：
 *   filtered = filtered + alpha * (raw - filtered)
 */
void RPM_Filter_Update(float raw_rpm)
{
    if (raw_rpm <= 0.0f)
    {
        motor_rpm_raw = 0.0f;
        motor_rpm_filtered = 0.0f;
        motor_rpm = 0.0f;
        return;
    }

    motor_rpm_raw = raw_rpm;

    if (motor_rpm_filtered <= 0.0f)
    {
        /*
         * 第一次从 0 进入有效测速时，直接赋值。
         * 避免滤波器从 0 慢慢爬升导致显示明显滞后。
         */
        motor_rpm_filtered = raw_rpm;
    }
    else
    {
        motor_rpm_filtered = motor_rpm_filtered +
                             RPM_FILTER_ALPHA * (raw_rpm - motor_rpm_filtered);
    }

    /*
     * 保持旧变量 motor_rpm 可用。
     * 当前 motor_rpm 代表滤波后的 RPM。
     */
    motor_rpm = motor_rpm_filtered;
}
