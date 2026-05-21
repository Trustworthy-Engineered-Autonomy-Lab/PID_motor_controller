#ifndef HALL_SPEED_H
#define HALL_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 霍尔输入捕获值：
 * 记录相邻霍尔边沿之间的定时器计数。
 */
extern volatile uint32_t hall_capture_value;

/*
 * 记录上一次霍尔传感器更新时间。
 */
extern volatile uint32_t lastHallSensorUpdate;

/*
 * 霍尔捕获异常跳变调试变量。
 */
extern volatile uint32_t debug_hall_capture_prev;
extern volatile uint32_t debug_hall_capture_delta;
extern volatile uint32_t debug_hall_capture_spike_count;

/*
 * 霍尔捕获值异常跳变检查。
 */
void hall_capture_spike_check(uint32_t capture_value);

/*
 * TIM3 输入捕获测速处理函数。
 *
 * 在 HAL_TIM_IC_CaptureCallback() 中调用。
 */
/*void Hall_Speed_Capture_Handler(TIM_HandleTypeDef *htim);*/
void hall_sensor_capture_handler(uint32_t capture_value);

/*
 * TIM3 溢出超时处理函数。
 *
 * 在 HAL_TIM_PeriodElapsedCallback() 的 TIM3 分支中调用。
 */
void hall_sensor_timeout_handler(void);

#ifdef __cplusplus
}
#endif

#endif
