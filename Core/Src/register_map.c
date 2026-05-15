#include "register_map.h"
#include "user.h"
#include "motor_control.h"

/*
 * 标志位目前仍然定义在 user.c。
 * 当寄存器被应用后，置 speed_update_flag = 1，
 * 表示收到新控制命令。
 */
extern volatile uint8_t speed_update_flag;

/*
 * 兼容旧版调试用变量。
 *
 * 当前控制逻辑不再直接依赖 speed_setpoint，
 * 但会把 target_pwm_us 同步给它，方便 CubeMonitor 观察。
 */
volatile int16_t speed_setpoint = 0;

/*
 * 软件寄存器镜像表。
 */
volatile uint8_t reg_map[REG_MAP_SIZE] = {0};

/*
 * 当前电机控制模式。
 */
volatile uint8_t motor_mode = MOTOR_MODE_STOP;

/*
 * 给 STM32CubeMonitor 观察用的模式显示变量。
 */
volatile uint16_t debug_motor_mode_view = 1000;

/*
 * 开环 PWM 目标脉宽，单位 us。
 */
volatile int16_t target_pwm_us = PWM_US_NEUTRAL;

/*
 * PID 目标转速，单位 RPM。
 */
volatile int16_t target_rpm = 0;

/*
 * 根据 motor_mode 更新 CubeMonitor 显示变量。
 */
void Update_Debug_Motor_Mode_View(void)
{
    switch (motor_mode)
    {
        case MOTOR_MODE_STOP:
            debug_motor_mode_view = 1000;
            break;

        case MOTOR_MODE_OPENLOOP_PWM:
            debug_motor_mode_view = 2000;
            break;

        case MOTOR_MODE_PID_RPM:
            debug_motor_mode_view = 3000;
            break;

        default:
            debug_motor_mode_view = 9999;
            break;
    }
}

/*
 * 初始化软件寄存器表 reg_map[] 的默认值。
 *
 * 上电后默认：
 *   REG_MODE = MOTOR_MODE_STOP
 *   REG_PWM_US = PWM_US_NEUTRAL
 *   REG_TARGET_RPM = 0
 */
void Register_Map_Init(void)
{
    for (uint8_t i = 0; i < REG_MAP_SIZE; i++) {
        reg_map[i] = 0;
    }

    reg_map[REG_MODE] = MOTOR_MODE_STOP;

    reg_map[REG_PWM_US_L] = (uint8_t)(PWM_US_NEUTRAL & 0xFF);
    reg_map[REG_PWM_US_H] = (uint8_t)(((uint16_t)PWM_US_NEUTRAL >> 8) & 0xFF);

    reg_map[REG_TARGET_RPM_L] = 0;
    reg_map[REG_TARGET_RPM_H] = 0;

    Register_Map_Apply();
}

/*
 * 将软件寄存器表 reg_map[] 中的原始字节数据，
 * 转换并同步到实际参与控制的变量。
 */
void Register_Map_Apply(void)
{
    /*
     * 1. 应用控制模式。
     */
    uint8_t old_mode = motor_mode;
    uint8_t mode = reg_map[REG_MODE];

    if (mode == MOTOR_MODE_STOP ||
        mode == MOTOR_MODE_OPENLOOP_PWM ||
        mode == MOTOR_MODE_PID_RPM)
    {
        motor_mode = mode;
    }
    else
    {
        motor_mode = MOTOR_MODE_STOP;
        reg_map[REG_MODE] = MOTOR_MODE_STOP;
    }

    /*
     * 模式切换时重置 PID，避免积分记忆跨模式保留。
     */
    if (old_mode != motor_mode) {
        if (old_mode == MOTOR_MODE_PID_RPM ||
            motor_mode == MOTOR_MODE_PID_RPM ||
            motor_mode == MOTOR_MODE_STOP)
        {
        	Motor_Control_Reset_PID();
        }
    }

    Update_Debug_Motor_Mode_View();

    /*
     * 2. 应用 PWM 脉宽。
     * 低字节在 REG_PWM_US_L，高字节在 REG_PWM_US_H。
     */
    int16_t pwm_value = (int16_t)((uint16_t)reg_map[REG_PWM_US_L] |
                                 ((uint16_t)reg_map[REG_PWM_US_H] << 8));

    target_pwm_us = PWM_US_CLAMP(pwm_value);

    /*
     * 同步旧变量，方便继续在 CubeMonitor 中观察 speed_setpoint。
     */
    speed_setpoint = target_pwm_us;

    /*
     * 如果输入 PWM 超出范围，reg_map 也同步修正为限幅后的值。
     */
    reg_map[REG_PWM_US_L] = (uint8_t)(target_pwm_us & 0xFF);
    reg_map[REG_PWM_US_H] = (uint8_t)(((uint16_t)target_pwm_us >> 8) & 0xFF);

    /*
     * 3. 应用目标 RPM。
     */
    target_rpm = (int16_t)((uint16_t)reg_map[REG_TARGET_RPM_L] |
                          ((uint16_t)reg_map[REG_TARGET_RPM_H] << 8));

    /*
     * 通知主循环：收到新的控制命令。
     */
    speed_update_flag = 1;
}
