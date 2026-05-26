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
 * Register addresses used by the I2C register interface.
 *
 * The host writes command values to these addresses. The STM32 stores
 * them in the internal register buffer in reg.c, and motor_control.c
 * reads them during each motor-control update.
 *
 * Register layout:
 *   REG_MODE           -> Motor control mode.
 *   REG_PWM_US_L/H     -> Open-loop PWM pulse width in microseconds,
 *                         stored as little-endian int16_t.
 *   REG_TARGET_RPM_L/H -> Target motor speed in RPM,
 *                         stored as little-endian int16_t.
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
 * These values are part of the register protocol. The host-side command
 * format, reg.c, and motor_control.c must use the same numeric values.
 */
#define REG_MOTOR_MODE_OPENLOOP_PWM        0
#define REG_MOTOR_MODE_PID_ACTIVE_BRAKE    1
#define REG_MOTOR_MODE_PID_RPM             2

/*
 * ============================================================
 * Default register values
 * ============================================================
 *
 * Default values loaded into the internal register buffer by reg_init().
 *
 * These defaults define the initial command state before the host writes
 * new values through the I2C register interface.
 */
#define REG_DEFAULT_MODE                   REG_MOTOR_MODE_OPENLOOP_PWM
#define REG_DEFAULT_PWM_US                 1500
#define REG_DEFAULT_TARGET_RPM             0

#ifdef __cplusplus
}
#endif

#endif /* REG_MAP_H */
