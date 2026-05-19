#ifndef REG_H
#define REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "reg_map.h"

void reg_init(void);

int read_reg(uint32_t addr, uint8_t *data, size_t len);
int write_reg(uint32_t addr, const uint8_t *data, size_t len);

/* I2C 中断转发接口 */
void I2C_LL_ResetRx(void);
void I2C_LL_RxByte(uint8_t data);
void I2C_LL_StopDetected(void);

#ifdef __cplusplus
}
#endif

#endif
