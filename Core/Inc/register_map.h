#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 软件寄存器地址定义 */
#define REG_MODE            0x01

#define REG_PWM_US_L        0x02
#define REG_PWM_US_H        0x03

#define REG_TARGET_RPM_L    0x04
#define REG_TARGET_RPM_H    0x05

/*
 * 软件寄存器表大小。
 * 当前有效寄存器地址为 0x01 ~ 0x05。
 * reg_map[0] 暂时不用。
 */
#define REG_MAP_SIZE        0x06

/* 控制模式定义 */
#define MOTOR_MODE_STOP          0
#define MOTOR_MODE_OPENLOOP_PWM  1
#define MOTOR_MODE_PID_RPM       2

/*
 * 软件寄存器表。
 */
extern volatile uint8_t reg_map[REG_MAP_SIZE];

/*
 * 当前电机控制模式。
 */
extern volatile uint8_t motor_mode;

/*
 * CubeMonitor 观察用模式变量：
 * 1000 = 停止模式
 * 2000 = 开环 PWM 模式
 * 3000 = PID RPM 模式
 * 9999 = 异常模式
 */
extern volatile uint16_t debug_motor_mode_view;

/*
 * 开环 PWM 目标脉宽，单位 us。
 */
extern volatile int16_t target_pwm_us;

/*
 * PID 目标转速，单位 RPM。
 */
extern volatile int16_t target_rpm;

/*
 * 兼容旧版调试变量。
 */
extern volatile int16_t speed_setpoint;

/*
 * 初始化软件寄存器表默认值。
 */
void Register_Map_Init(void);

/*
 * 将 reg_map[] 中的字节数据同步到实际控制变量。
 */
void Register_Map_Apply(void);

/*
 * 根据 motor_mode 更新 debug_motor_mode_view。
 */
void Update_Debug_Motor_Mode_View(void);

#ifdef __cplusplus
}
#endif

#endif
