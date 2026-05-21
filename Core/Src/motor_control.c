#include <lp_filter.h>
#include "motor_control.h"
#include "app_config.h"
#include "reg.h"
#include "pid.h"
#include "rpm_filter.h"

extern TIM_HandleTypeDef htim2;

/* PWM */
volatile int16_t debug_pwm_us = PWM_US_NEUTRAL;
volatile uint32_t debug_pwm_ccr = 0;

uint32_t Motor_Control_PWM_UsToCcr(int16_t pulse_us)
{
    pulse_us = PWM_US_CLAMP(pulse_us);
    return PWM_US_TO_CCR(pulse_us);
}

void Motor_Control_Set_PWM_US(int16_t pulse_us)
{
    pulse_us = PWM_US_CLAMP(pulse_us);

    uint32_t ccr = PWM_US_TO_CCR(pulse_us);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);

    debug_pwm_us = pulse_us;
    debug_pwm_ccr = ccr;
}

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

volatile uint8_t motor_mode = MOTOR_MODE_OPENLOOP_PWM;
volatile int16_t target_pwm_us = PWM_US_NEUTRAL;
volatile int16_t target_rpm = 0;
volatile int16_t speed_setpoint = PWM_US_NEUTRAL;

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
const float Kp = 0.00002f;
const float Ki = 0.00003f;
const float Kd = 0.0f;
const float Integral_max = 100000.0f;
const float pid_max = PWM_MAX_PULSEWIDTH - PWM_ZERO_PULSEWIDTH;

/*
 * 电机控制初始化。
 */
void Motor_Control_Init(void)
{
    motor_mode = MOTOR_MODE_OPENLOOP_PWM;
    target_pwm_us = PWM_US_NEUTRAL;
    target_rpm = 0;
    speed_setpoint = PWM_US_NEUTRAL;

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

static void Motor_Control_Read_Registers(void)
{
    uint8_t mode = MOTOR_MODE_OPENLOOP_PWM;
    uint8_t buf[2] = {0};

    if (read_reg(REG_MODE, &mode, 1) == 1)
    {
        if (mode == MOTOR_MODE_OPENLOOP_PWM ||
            mode == MOTOR_MODE_PID_ACTIVE_BRAKE ||
            mode == MOTOR_MODE_PID_RPM)
        {
            motor_mode = mode;
        }
        else
        {
            motor_mode = MOTOR_MODE_OPENLOOP_PWM;
        }
    }

    if (read_reg(REG_PWM_US_L, buf, 2) == 2)
    {
        int16_t pwm = (int16_t)(
            ((uint16_t)buf[1] << 8) |
            ((uint16_t)buf[0])
        );

        if (pwm == 0)
        {
            target_pwm_us = PWM_US_NEUTRAL;
        }
        else
        {
            target_pwm_us = PWM_US_CLAMP(pwm);
        }
    }

    if (read_reg(REG_TARGET_RPM_L, buf, 2) == 2)
    {
        target_rpm = (int16_t)(
            ((uint16_t)buf[1] << 8) |
            ((uint16_t)buf[0])
        );
    }

    if (motor_mode == MOTOR_MODE_OPENLOOP_PWM)
    {
        speed_setpoint = target_pwm_us;
    }
    else
    {
        speed_setpoint = target_rpm;
    }
}

/*
 * 电机控制周期更新函数。
 *
 * 由 User_Loop() 在 TIM1 控制周期中调用。
 */
void Motor_Control_Update(void)
{
    Motor_Control_Read_Registers();

    switch (motor_mode)
    {
        case MOTOR_MODE_OPENLOOP_PWM:
        {
            Motor_Control_Set_PWM_US(target_pwm_us);
            break;
        }

        case MOTOR_MODE_PID_ACTIVE_BRAKE:
        {
            pid_pwm_update((float)target_rpm);
            break;
        }

        case MOTOR_MODE_PID_RPM:
        {
            pid_pwm_update((float)target_rpm);
            break;
        }

        default:
        {
            motor_mode = MOTOR_MODE_OPENLOOP_PWM;
            target_pwm_us = PWM_US_NEUTRAL;
            Motor_Control_Set_PWM_US(PWM_US_NEUTRAL);
            break;
        }
    }
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

	    float feedback_rpm = motor_rpm;
	    int16_t pulse_us;

	    /*
	     * PID 主动刹车模式：
	     * 速度高时主动刹车，速度低时回到 neutral。
	     */
	    if (motor_mode == MOTOR_MODE_PID_ACTIVE_BRAKE) {

	        static uint8_t brake_active = 0;

	        if (brake_active == 0) {
	            if (feedback_rpm > MOTOR_BRAKE_ON_RPM) {
	                brake_active = 1;
	            }
	        } else {
	            if (feedback_rpm < MOTOR_BRAKE_OFF_RPM) {
	                brake_active = 0;
	            }
	        }

	        if (brake_active) {
	            pulse_us = MOTOR_BRAKE_PWM_US;
	        } else {
	            pulse_us = PWM_US_NEUTRAL;
	        }
	    }

	    /*
	     * PID 自然停止模式：
	     * 不主动刹车，直接输出 neutral，让电机自然滑停。
	     */
	    else {
	        pulse_us = PWM_US_NEUTRAL;
	    }

	    Motor_Control_Set_PWM_US(pulse_us);

	    debug_motor_rpm = feedback_rpm;
	    debug_pid_output = 0.0f;
	    debug_pid_error = 0.0f;

	    return;
	}

	/* 计算 PID 输出。motor_pid.output 表示相对于中位脉宽的修正量。 */
	float feedback_rpm = motor_rpm;
	PID_Compute(&motor_pid, rpm_setpoint, feedback_rpm, dt);
	debug_motor_rpm = feedback_rpm;
	debug_pid_output = motor_pid.output;
	debug_pid_error = rpm_setpoint - feedback_rpm;

	/* 将 PID 输出转换为实际 PWM 脉宽，再转换为 CCR。 */
	float pulse_width = PWM_ZERO_PULSEWIDTH + motor_pid.output;

	/*
	 * 当前未加入反转逻辑，所以低于中位值的输出被限制到中位值。
	 * 后续如果需要电机反转，需要重新设计正转 / 停止 / 反转状态机。
	 */
	if (pulse_width < PWM_ZERO_PULSEWIDTH) {
	    pulse_width = PWM_ZERO_PULSEWIDTH;
	} else if (pulse_width > PWM_MAX_PULSEWIDTH) {
	    pulse_width = PWM_MAX_PULSEWIDTH;
	}

	Motor_Control_Set_PWM_US((int16_t)(pulse_width * 1000.0f));
}
