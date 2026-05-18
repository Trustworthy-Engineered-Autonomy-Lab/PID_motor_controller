#include "hall_speed.h"
#include "user.h"
#include "rpm_filter.h"

/*
 * hall_update_flag 目前仍然定义在 user.c。
 * 霍尔捕获或超时时，需要置位这个标志。
 */
extern volatile uint8_t hall_update_flag;

/*
 * 霍尔输入捕获值：
 * 记录相邻霍尔边沿之间的定时器计数。
 */
volatile uint32_t hall_capture_value = 0;

/*
 * 记录上一次霍尔传感器更新时间。
 */
volatile uint32_t lastHallSensorUpdate = 0;

/*
 * 霍尔捕获异常跳变调试变量。
 */
volatile uint32_t debug_hall_capture_prev = 0;
volatile uint32_t debug_hall_capture_delta = 0;
volatile uint32_t debug_hall_capture_spike_count = 0;

/*
 * 霍尔捕获值异常跳变检查。
 *
 * 该函数只用于调试，不参与控制。
 *
 * 如果 hall_capture_value 相比上一帧突然变得过大或过小，
 * 说明可能存在霍尔边沿干扰、漏捕获、低速不均匀或机械周期性波动。
 */
void Hall_Capture_Spike_Check(uint32_t capture_value)
{
    if (debug_hall_capture_prev > 0)
    {
        if (capture_value > debug_hall_capture_prev)
        {
            debug_hall_capture_delta = capture_value - debug_hall_capture_prev;
        }
        else
        {
            debug_hall_capture_delta = debug_hall_capture_prev - capture_value;
        }

        /*
         * 简单异常判断：
         * 当前捕获值小于上一次的一半，或大于上一次的 2 倍，
         * 就记录为一次明显跳变。
         */
        if ((capture_value < (debug_hall_capture_prev / 2)) ||
            (capture_value > (debug_hall_capture_prev * 2)))
        {
            debug_hall_capture_spike_count++;
        }
    }

    debug_hall_capture_prev = capture_value;
}

/*
 * TIM3 输入捕获测速处理函数。
 *
 * 当 TIM3 捕获到霍尔传感器边沿时调用。
 */
void Hall_Speed_Capture_Handler(TIM_HandleTypeDef *htim)
{
    hall_update_flag = 1;

    /*
     * 读取 TIM3 CCR1，获得本次霍尔边沿对应的捕获值。
     * hall_capture_value 表示相邻霍尔边沿之间的定时器计数。
     */
    hall_capture_value = htim->Instance->CCR1;

    /*
     * 检查 hall_capture_value 是否出现明显异常跳变。
     * 该检查只用于 CubeMonitor 调试观察，不参与控制。
     */
    Hall_Capture_Spike_Check(hall_capture_value);

    float raw_rpm_now;

    if (motor_rpm_raw > 0.0f || motor_rpm_filtered > 0.0f)
    {
        raw_rpm_now = HALL_CAPTURE_TO_RPM(hall_capture_value);
    }
    else
    {
        /*
         * 电机刚从停止进入转动时，第一次捕获可能异常。
         * 保留原来的 MIN_MOTOR_RPM 保护逻辑。
         */
        raw_rpm_now = MIN_MOTOR_RPM;
    }

    /*
     * 霍尔中断只保存最新原始 RPM，不在这里执行滤波。
     * 滤波仍然在 TIM1 控制周期中执行。
     */
    latest_raw_rpm = raw_rpm_now;
    motor_rpm_raw = raw_rpm_now;
    lastHallSensorUpdate = HAL_GetTick();
}

/*
 * TIM3 溢出超时处理函数。
 *
 * 如果 TIM3 溢出时 hall_update_flag == 0，
 * 说明较长时间没有检测到霍尔边沿，可以认为电机停止。
 */
void Hall_Speed_Timeout_Handler(void)
{
    if (hall_update_flag == 0)
    {
        /*
         * 没有输入捕获却发生 TIM3 溢出，
         * 说明长时间没有霍尔边沿，认为电机停止。
         */
        latest_raw_rpm = 0.0f;
        motor_rpm_raw = 0.0f;
        motor_rpm_filtered = 0.0f;
        motor_rpm = 0.0f;

        hall_update_flag = 1;
    }
}
