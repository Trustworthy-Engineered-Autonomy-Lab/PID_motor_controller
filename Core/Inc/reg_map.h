#ifndef REG_MAP_H
#define REG_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================
 * Register address map
 * ============================================================
 *
 * 上位机通过 I2C 写入这些寄存器，STM32 在控制周期中读取寄存器值。
 *
 * 当前寄存器定义：
 *   REG_MODE          : 电机控制模式
 *   REG_PWM_US_L/H    : 开环 PWM 脉宽，单位 us，小端格式
 *   REG_TARGET_RPM_L/H: PID 目标转速，单位 RPM，小端格式
 */
typedef enum
{
    REG_MODE = 0x00,

    REG_PWM_US_L = 0x01,
    REG_PWM_US_H = 0x02,

    REG_TARGET_RPM_L = 0x03,
    REG_TARGET_RPM_H = 0x04,

    REG_COUNT
} RegAddr_t;


/*
 * ============================================================
 * Motor mode values stored in REG_MODE
 * ============================================================
 *
 * 注意：
 * 这些值是“寄存器协议”的一部分。
 * reg.c、motor_control.c、上位机都应该使用同一套数值。
 */
#define REG_MOTOR_MODE_OPENLOOP_PWM        0
#define REG_MOTOR_MODE_PID_ACTIVE_BRAKE    1
#define REG_MOTOR_MODE_PID_RPM             2


/*
 * ============================================================
 * Default register values
 * ============================================================
 *
 * 这些默认值只表示寄存器上电后的初始状态。
 * reg.c 可以使用它们，而不需要包含 motor_control.h。
 */
#define REG_DEFAULT_MODE                   REG_MOTOR_MODE_OPENLOOP_PWM
#define REG_DEFAULT_PWM_US                 1500
#define REG_DEFAULT_TARGET_RPM             0

#ifdef __cplusplus
}
#endif

#endif
