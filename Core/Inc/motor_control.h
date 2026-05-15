#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * PID 调试变量。
 */
extern volatile float debug_motor_rpm;
extern volatile float debug_pid_output;
extern volatile float debug_pid_error;

/*
 * 电机控制初始化。
 *
 * 当前主要用于初始化 PID 控制器。
 */
void Motor_Control_Init(void);

/*
 * 电机控制周期更新函数。
 *
 * 在 User_Loop() 的 TIM1 控制周期中调用。
 * 根据 motor_mode 选择：
 *   STOP
 *   OPENLOOP_PWM
 *   PID_RPM
 */
void Motor_Control_Update(void);

/*
 * 重置电机 PID 控制器。
 *
 * 外部模块如果需要在模式切换时清空 PID 积分项，
 * 应该调用这个函数，而不是直接访问 motor_pid。
 */
void Motor_Control_Reset_PID(void);

/*
 * 速度 / 转速 / PWM 转换相关函数。
 */
float rpm_update(int16_t speed_setpoint);
void openloop_pwm_update(float rpm_setpoint);
void pid_pwm_update(float rpm_setpoint);
uint32_t rpm_to_pwm_duty(int16_t rpm_setpoint);

#ifdef __cplusplus
}
#endif

#endif
