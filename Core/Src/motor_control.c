#include "motor_control.h"
#include "app_config.h"
#include "reg.h"
#include "pid.h"
#include "lp_filter.h"

/*
 * PWM output debug values.
 */
volatile int16_t debug_pwm_us = PWM_US_NEUTRAL;
volatile uint32_t debug_pwm_ccr = 0;

/*
 * Converts a PWM pulse width in microseconds to a TIM2 CCR value.
 *
 * The input is clamped to the configured valid PWM range before
 * conversion.
 */
uint32_t motor_control_pwm_us_to_ccr(int16_t pulse_us)
{
    pulse_us = PWM_US_CLAMP(pulse_us);
    return PWM_US_TO_CCR(pulse_us);
}

/*
 * Applies a PWM pulse-width command to the motor PWM timer.
 *
 * The input pulse width is clamped before conversion. The resulting CCR
 * value is written to the configured PWM timer channel, and the debug
 * variables are updated with the applied values.
 */
void motor_control_set_pwm_us(int16_t pulse_us)
{
    pulse_us = PWM_US_CLAMP(pulse_us);

    uint32_t ccr = PWM_US_TO_CCR(pulse_us);

    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER_HANDLE, MOTOR_PWM_CHANNEL, ccr);

    debug_pwm_us = pulse_us;
    debug_pwm_ccr = ccr;
}

/*
 * PID controller instance used by the motor-control module.
 */
PID_t motor_pid;

/*
 * PID update debug values.
 */
volatile float debug_motor_rpm = 0.0f;
volatile float debug_pid_output = 0.0f;
volatile float debug_pid_error = 0.0f;

/*
 * Current motor command state.
 *
 * motor_mode selects the active control mode.
 * target_pwm_us stores the open-loop PWM command.
 * target_rpm stores the PID speed command.
 * speed_setpoint mirrors target_pwm_us in open-loop mode and target_rpm
 * in PID modes. The current control logic uses target_pwm_us and
 * target_rpm directly.
 */
volatile uint8_t motor_mode = MOTOR_MODE_OPENLOOP_PWM;
volatile int16_t target_pwm_us = PWM_US_NEUTRAL;
volatile int16_t target_rpm = 0;
volatile int16_t speed_setpoint = PWM_US_NEUTRAL;

/*
 * Raw RPM feedback value.
 *
 * This value is updated by the Hall sensor path and by the control-loop
 * filter synchronization logic.
 */
volatile float motor_rpm_raw = 0.0f;

/*
 * Filtered RPM feedback value.
 *
 * This value is produced by the RPM low-pass filter in the control loop.
 * The Hall timeout handler may also clear it when no Hall edge is
 * detected during a timeout window.
 */
volatile float motor_rpm_filtered = 0.0f;

/*
 * Backward-compatible RPM feedback variable.
 *
 * In the current project, this value is synchronized with
 * motor_rpm_filtered and is used as the feedback input for PID control.
 */
volatile float motor_rpm = 0.0f;

/*
 * Latest raw RPM sample from the Hall capture path.
 *
 * The Hall capture handler updates this value. The control loop passes
 * it into the low-pass filter during each control update.
 */
volatile float latest_raw_rpm = 0.0f;

/*
 * RPM filter update debug counter.
 *
 * This counter is incremented each time the control loop computes one
 * RPM filter update.
 */
volatile uint32_t debug_filter_update_count = 0;

/*
 * Latest RPM setpoint passed to pid_pwm_update().
 *
 * This variable is internal to this module and mirrors the function
 * input used by the PID update logic.
 */
static volatile float rpm_setpoint = 0.0f;

/*
 * PID configuration values.
 *
 * dt is derived from the TIM1 control period.
 * kp, ki, and kd are the PID gains.
 * integral_max limits the accumulated PID error.
 * pid_max limits the PID output around the neutral PWM pulse width.
 */
const float dt = TIM1_PER_MS / 1000.0f;
const float kp = 0.00002f;
const float ki = 0.00003f;
const float kd = 0.0f;
const float integral_max = 100000.0f;
const float pid_max = PWM_MAX_PULSEWIDTH - PWM_ZERO_PULSEWIDTH;

/*
 * Initializes the motor-control module.
 *
 * The command state is reset to open-loop neutral output, and the PID
 * controller is initialized with the configured gains and limits.
 */
void motor_control_init(void)
{
    motor_mode = MOTOR_MODE_OPENLOOP_PWM;
    target_pwm_us = PWM_US_NEUTRAL;
    target_rpm = 0;
    speed_setpoint = PWM_US_NEUTRAL;

    pid_init(&motor_pid, kp, ki, kd, integral_max, pid_max);
}

/*
 * Resets the motor PID controller state.
 *
 * External modules should call this function when the PID memory must be
 * cleared, instead of accessing motor_pid directly.
 */
void motor_control_reset_pid(void)
{
    pid_reset(&motor_pid);
}

/*
 * Reads motor command registers and updates the local command state.
 *
 * Invalid motor modes fall back to open-loop PWM mode. Invalid open-loop
 * PWM pulse-width commands fall back to the neutral pulse width.
 */
static void motor_control_read_registers(void)
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

        if (pwm < PWM_US_MIN || pwm > PWM_US_MAX)
        {
            target_pwm_us = PWM_US_NEUTRAL;
        }
        else
        {
            target_pwm_us = pwm;
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
 * Performs one motor-control update.
 *
 * The function first refreshes the local command state from the register
 * buffer, then applies the selected control mode.
 */
void motor_control_update(void)
{
    motor_control_read_registers();

    switch (motor_mode)
    {
        case MOTOR_MODE_OPENLOOP_PWM:
        {
            motor_control_set_pwm_us(target_pwm_us);
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
            motor_control_set_pwm_us(PWM_US_NEUTRAL);
            break;
        }
    }
}

/*
 * Updates the PWM command for PID-based RPM control.
 *
 * For positive RPM setpoints, the function computes a PID correction
 * from the current RPM feedback and applies it around the neutral PWM
 * pulse width.
 *
 * For zero or negative RPM setpoints, the PID controller is reset. In
 * active-brake mode, the function applies hysteretic braking based on
 * the current RPM feedback. In the non-braking PID mode, it outputs the
 * neutral pulse width.
 */
void pid_pwm_update(float rpm_setpoint_input)
{
	rpm_setpoint = rpm_setpoint_input;
	if (rpm_setpoint <= 0.0f) {

	    pid_reset(&motor_pid);

	    float feedback_rpm = motor_rpm;
	    int16_t pulse_us;

	    /*
	     * Active-brake stop behavior.
	     *
	     * Braking is enabled when the feedback RPM rises above the brake-on
	     * threshold and disabled when it falls below the brake-off threshold.
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
	     * Non-braking stop behavior.
	     *
	     * The PWM output is set to neutral so the motor can coast down
	     * without an active braking pulse.
	     */
	    else {
	        pulse_us = PWM_US_NEUTRAL;
	    }

	    motor_control_set_pwm_us(pulse_us);

	    debug_motor_rpm = feedback_rpm;
	    debug_pid_output = 0.0f;
	    debug_pid_error = 0.0f;

	    return;
	}

	/*
	 * Compute the PID output. motor_pid.output is the pulse-width
	 * correction relative to the neutral PWM pulse width.
	 */
	float feedback_rpm = motor_rpm;
	pid_compute(&motor_pid, rpm_setpoint, feedback_rpm, dt);
	debug_motor_rpm = feedback_rpm;
	debug_pid_output = motor_pid.output;
	debug_pid_error = rpm_setpoint - feedback_rpm;

	/*
	 * Convert the PID correction to an actual PWM pulse width.
	 */
	float pulse_width = PWM_ZERO_PULSEWIDTH + motor_pid.output;

	/*
	 * This project currently does not implement reverse motor control.
	 * Any PID result below the neutral pulse width is limited to neutral.
	 */
	if (pulse_width < PWM_ZERO_PULSEWIDTH) {
	    pulse_width = PWM_ZERO_PULSEWIDTH;
	} else if (pulse_width > PWM_MAX_PULSEWIDTH) {
	    pulse_width = PWM_MAX_PULSEWIDTH;
	}

	motor_control_set_pwm_us((int16_t)(pulse_width * 1000.0f));
}
