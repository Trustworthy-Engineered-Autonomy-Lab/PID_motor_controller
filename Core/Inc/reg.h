#ifndef REG_H
#define REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "reg_map.h"

/*
 * Initializes the internal register buffer and the LL I2C slave interface.
 *
 * This function clears the register buffer and I2C receive buffer, then
 * enables the I2C1 event, buffer, and error interrupts. Module-specific
 * default register values should be written by the modules that own them.
 */
void reg_init(void);

/*
 * Reads bytes from the internal register buffer.
 *
 * addr is the first register address to read.
 * data points to the destination buffer.
 * len is the requested number of bytes.
 *
 * Returns the number of bytes copied on success. Returns a negative
 * errno-style value if the destination pointer is null or the start
 * address is outside the register map. If the requested range extends
 * past REG_COUNT, the length is clipped to the valid register range.
 */
int read_reg(uint32_t addr, uint8_t *data, size_t len);

/*
 * Writes bytes to the internal register buffer.
 *
 * addr is the first register address to write.
 * data points to the source buffer.
 * len is the requested number of bytes.
 *
 * Returns the number of bytes copied on success. Returns a negative
 * errno-style value if the source pointer is null or the start address
 * is outside the register map. If the requested range extends past
 * REG_COUNT, the length is clipped to the valid register range.
 */
int write_reg(uint32_t addr, const uint8_t *data, size_t len);

/*
 * LL I2C receive-path forwarding functions.
 *
 * These functions are called from the I2C1 event interrupt handler.
 * i2c_ll_reset_rx() clears the temporary receive length.
 * i2c_ll_rx_byte() appends one received byte to the temporary buffer.
 * i2c_ll_stop_detected() commits a completed write frame to the internal
 * register buffer through write_reg().
 */
void i2c_ll_reset_rx(void);
void i2c_ll_rx_byte(uint8_t data);
void i2c_ll_stop_detected(void);

#ifdef __cplusplus
}
#endif

#endif /* REG_H */
