#include "motor_control.h"
#include "user.h"
#include "pid.h"
#include "register_map.h"
#include "rpm_filter.h"
#include "pwm_output.h"

/*
 * 电机 PID 控制器实例。
 */
PID_t motor_pid;

/*
 * PID 调试变量。
 */
volatile float debug_motor_rpm = 0.0f;
volatile float debug_pid_output = 0.0f;
volatile float debug_pid_error = 0.0f;

/*
 * PID 闭环模式下的目标转速。
 *
 * 当前主要使用 target_rpm。
 * rpm_setpoint 暂时保留，后续如果完全不用可以删除。
 */
static volatile float rpm_setpoint = 0.0f;

/*
 * PID 参数。
 */
const float dt = TIM1_PER_MS / 1000.0f;
const float Kp = 0.00001f;
const float Ki = 0.000005f;
const float Kd = 0.0f;
const float Integral_max = 100000.0f;
const float pid_max = PWM_MAX_PULSEWIDTH - PWM_ZERO_PULSEWIDTH;

/*
 * 电机控制初始化。
 */
void Motor_Control_Init(void)
{
    PID_Init(&motor_pid, Kp, Ki, Kd, Integral_max, pid_max);
}

/*
 * 重置电机 PID 控制器。
 *
 * 这个函数提供给 register_map.c 等外部模块调用，
 * 外部模块不需要也不应该直接访问 motor_pid。
 */
void Motor_Control_Reset_PID(void)
{
    PID_Reset(&motor_pid);
}

/*
 * 电机控制周期更新函数。
 *
 * 由 User_Loop() 在 TIM1 控制周期中调用。
 */
void Motor_Control_Update(void)
{
    switch (motor_mode)
    {
        case MOTOR_MODE_STOP:
        {
            PWM_Output_Set_US(PWM_US_NEUTRAL);
            break;
        }

        case MOTOR_MODE_OPENLOOP_PWM:
        {
            PWM_Output_Set_US(target_pwm_us);
            break;
        }

        case MOTOR_MODE_PID_RPM:
        {
            pid_pwm_update((float)target_rpm);
            break;
        }

        default:
        {
            motor_mode = MOTOR_MODE_STOP;
            Update_Debug_Motor_Mode_View();

            PWM_Output_Set_US(PWM_US_NEUTRAL);
            break;
        }
    }
}

/*
 * 将上位机输入的速度类指令转换为目标 RPM。
 *
 * 当前版本中，目标 RPM 主要通过：
 *   REG_TARGET_RPM_L
 *   REG_TARGET_RPM_H
 * 写入，并在 Register_Map_Apply() 中组合成 target_rpm。
 *
 * 因此该函数目前不是主控制路径的一部分，只作为旧版接口保留。
 */
float rpm_update(int16_t speed_setpoint)
{
    return (float)speed_setpoint;
}

/*
 * 旧版开环 PWM 更新函数，当前主控制路径未调用。
 *
 * 注意：
 *   这里的参数名 rpm_setpoint 容易误导。
 *   在该函数内部，它实际被当作 CCR 值使用，并不是真正的 RPM。
 */
void openloop_pwm_update(float rpm_setpoint)
{
    uint32_t new_ccr_value;

    if (rpm_setpoint < PWM_CCR_MIN) {
        new_ccr_value = PWM_CCR_MIN;
    } else if (rpm_setpoint > PWM_CCR_MAX) {
        new_ccr_value = PWM_CCR_MAX;
    } else {
        new_ccr_value = (uint32_t)rpm_setpoint;
    }

    /*
     * 旧函数保留原逻辑：直接写 TIM2 CCR。
     * 当前推荐主路径使用 PWM_Output_Set_US()。
     */
    extern TIM_HandleTypeDef htim2;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, new_ccr_value);
}

/*
 * PID 闭环 PWM 更新函数。
 *
 * 在 MOTOR_MODE_PID_RPM 模式下调用。
 */
void pid_pwm_update(float rpm_setpoint_input)
{
    rpm_setpoint = rpm_setpoint_input;

    if (rpm_setpoint <= 0.0f) {
        PID_Reset(&motor_pid);

        PWM_Output_Set_US(PWM_US_NEUTRAL);

        debug_motor_rpm = motor_rpm;
        debug_pid_output = 0.0f;
        debug_pid_error = 0.0f;

        return;
    }

    /*
     * 计算 PID 输出。
     * motor_pid.output 表示相对于中位脉宽的修正量。
     */
    float feedback_rpm = motor_rpm;
    PID_Compute(&motor_pid, rpm_setpoint, feedback_rpm, dt);

    debug_motor_rpm = feedback_rpm;
    debug_pid_output = motor_pid.output;
    debug_pid_error = rpm_setpoint - feedback_rpm;

    /*
     * 将 PID 输出转换为实际 PWM 脉宽。
     *
     * 注意：
     *   PWM_ZERO_PULSEWIDTH 和 PWM_MAX_PULSEWIDTH 当前是 ms 单位，
     *   例如 1.5、2.0。
     */
    float pulse_width = PWM_ZERO_PULSEWIDTH + motor_pid.output;

    /*
     * 当前未加入反转逻辑，所以低于中位值的输出被限制到中位值。
     */
    if (pulse_width < PWM_ZERO_PULSEWIDTH) {
        pulse_width = PWM_ZERO_PULSEWIDTH;
    } else if (pulse_width > PWM_MAX_PULSEWIDTH) {
        pulse_width = PWM_MAX_PULSEWIDTH;
    }

    uint32_t ccr_output = (uint32_t)PWM_PULSEWIDTH_TO_CCR(pulse_width);

    extern TIM_HandleTypeDef htim2;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_output);

    debug_pwm_us = (int16_t)(pulse_width * 1000.0f);
    debug_pwm_ccr = ccr_output;
}

/*
 * RPM 到 PWM CCR 的映射函数，当前未实现。
 *
 * 当前函数直接 return 0，不应在实际控制中调用。
 */
uint32_t rpm_to_pwm_duty(int16_t rpm_setpoint)
{
    /*
     * 后续可以通过实验记录：
     *   PWM 脉宽 / CCR 与实际 RPM 的对应关系，
     * 再用线性拟合或查表方式实现该函数。
     */
    return 0;
}
