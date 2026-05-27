#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "reg_map.h"

/*
 * Motor control mode aliases.
 *
 * These values are mapped directly to the REG_MODE register protocol.
 * Keep them synchronized with the register definitions in reg_map.h.
 */
#define MOTOR_MODE_OPENLOOP_PWM       REG_MOTOR_MODE_OPENLOOP_PWM
#define MOTOR_MODE_PID_ACTIVE_BRAKE   REG_MOTOR_MODE_PID_ACTIVE_BRAKE
#define MOTOR_MODE_PID_RPM            REG_MOTOR_MODE_PID_RPM

/*
 * PWM output debug variables.
 *
 * debug_pwm_us stores the latest clamped PWM pulse width in microseconds.
 * debug_pwm_ccr stores the latest CCR value written to the PWM timer.
 */
extern volatile int16_t debug_pwm_us;
extern volatile uint32_t debug_pwm_ccr;

/*
 * Converts a PWM pulse width in microseconds to a timer CCR value.
 *
 * The input pulse width is clamped to the configured valid PWM range
 * before conversion.
 */
uint32_t motor_control_pwm_us_to_ccr(int16_t pulse_us);

/*
 * Sets the motor PWM output pulse width in microseconds.
 *
 * The input pulse width is clamped to the configured valid PWM range.
 * The function updates both the timer compare register and the PWM
 * debug variables.
 */
void motor_control_set_pwm_us(int16_t pulse_us);

/*
 * PID control debug variables.
 *
 * debug_motor_rpm stores the RPM feedback value used by the PID update.
 * debug_pid_output stores the latest PID output value.
 * debug_pid_error stores the latest setpoint-feedback error.
 */
extern volatile float debug_motor_rpm;
extern volatile float debug_pid_output;
extern volatile float debug_pid_error;

/*
 * Active-brake parameters for zero-RPM commands in active-brake mode.
 *
 * MOTOR_BRAKE_PWM_US is the braking pulse width.
 * MOTOR_BRAKE_ON_RPM enables braking above this RPM threshold.
 * MOTOR_BRAKE_OFF_RPM disables braking below this RPM threshold.
 */
#define MOTOR_BRAKE_PWM_US 1480
#define MOTOR_BRAKE_ON_RPM 80.0f
#define MOTOR_BRAKE_OFF_RPM 30.0f

/*
 * Initializes the motor-control module.
 *
 * This function resets the motor command state to the default open-loop
 * PWM mode and initializes the PID controller.
 */
void motor_control_init(void);

/*
 * Performs one motor-control update.
 *
 * This function reads the command registers, updates the active motor
 * mode, and applies the selected control behavior:
 *   MOTOR_MODE_OPENLOOP_PWM     -> direct PWM pulse-width command.
 *   MOTOR_MODE_PID_ACTIVE_BRAKE -> PID speed control with active braking
 *                                  when the target RPM is zero or below.
 *   MOTOR_MODE_PID_RPM          -> PID speed control with neutral output
 *                                  when the target RPM is zero or below.
 *
 * In the current project, user_loop() calls this function when the
 * control update flag is set by the control timer.
 */
void motor_control_update(void);

/*
 * Resets the motor PID controller state.
 *
 * External modules should use this function when the PID integral or
 * derivative history must be cleared, instead of accessing motor_pid
 * directly.
 */
void motor_control_reset_pid(void);

/*
 * Updates the PWM command for PID-based RPM control.
 *
 * For positive RPM setpoints, this function computes the PID correction
 * from the current RPM feedback and applies the resulting PWM command.
 * For zero or negative RPM setpoints, it resets the PID controller and
 * applies the stop behavior selected by the current motor mode.
 */
void pid_pwm_update(float rpm_setpoint);

/*
 * Motor-control default command values.
 *
 * These defaults belong to the motor-control module because they define
 * the initial motor-control command state stored in the register map.
 * reg.c only provides register storage and access.
 */
#define MOTOR_DEFAULT_MODE        MOTOR_MODE_OPENLOOP_PWM
#define MOTOR_DEFAULT_PWM_US      PWM_US_NEUTRAL
#define MOTOR_DEFAULT_TARGET_RPM  0

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
