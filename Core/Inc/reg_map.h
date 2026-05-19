#ifndef REG_MAP_H
#define REG_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    REG_MODE = 0x00,

    REG_PWM_US_L = 0x01,
    REG_PWM_US_H = 0x02,

    REG_TARGET_RPM_L = 0x03,
    REG_TARGET_RPM_H = 0x04,

    REG_COUNT
} RegAddr_t;

#ifdef __cplusplus
}
#endif

#endif
