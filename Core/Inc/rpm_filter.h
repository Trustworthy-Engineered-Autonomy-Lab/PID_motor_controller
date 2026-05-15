#ifndef RPM_FILTER_H
#define RPM_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * RPM 一阶低通滤波系数。
 *
 * alpha 越大，响应越快，但波动越明显；
 * alpha 越小，曲线越平滑，但响应更慢。
 *
 * 当前建议值：0.05f
 */
#define RPM_FILTER_ALPHA 0.05f

/*
 * 霍尔测速得到的原始瞬时 RPM。
 */
extern volatile float motor_rpm_raw;

/*
 * 滤波后的 RPM。
 */
extern volatile float motor_rpm_filtered;

/*
 * 兼容旧变量名。
 *
 * 当前 motor_rpm 同步为 motor_rpm_filtered。
 * PID 控制中使用 motor_rpm 作为反馈值。
 */
extern volatile float motor_rpm;

/*
 * 最新一次霍尔捕获得到的原始 RPM。
 *
 * 霍尔中断更新这个变量；
 * TIM1 控制周期中调用 RPM_Filter_Update() 进行滤波。
 */
extern volatile float latest_raw_rpm;

/*
 * 调试用：观察滤波器是否按 TIM1 周期运行。
 */
extern volatile uint32_t debug_filter_update_count;

/*
 * RPM 一阶低通滤波更新函数。
 */
void RPM_Filter_Update(float raw_rpm);

#ifdef __cplusplus
}
#endif

#endif
