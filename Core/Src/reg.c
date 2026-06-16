#include "reg.h"
#include "main.h"
#include <string.h>
#include <errno.h>

/*
 * I2C receive buffer size.
 *
 * One byte is used for the start register address. The remaining bytes
 * can contain a full register-map write payload.
 */
#define I2C_RX_BUF_SIZE (REG_COUNT + 1)

/*
 * Internal register buffer and temporary I2C receive buffer.
 */
static uint8_t reg[REG_COUNT] = {0};
static uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE] = {0};
static uint16_t i2c_rx_len = 0;

/*
 * Initializes the register module and enables the LL I2C slave interface.
 *
 * The internal register buffer and temporary I2C receive buffer are
 * cleared. Module-specific default register values are initialized by
 * the modules that own those commands.
 */
void reg_init(void)
{
    memset((void *)reg, 0, sizeof(reg));
    memset((void *)i2c_rx_buf, 0, sizeof(i2c_rx_buf));
    i2c_rx_len = 0;

    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
    LL_I2C_EnableIT_TX(I2C1);
    LL_I2C_EnableIT_RX(I2C1);
    LL_I2C_EnableIT_TC(I2C1);
    LL_I2C_EnableIT_STOP(I2C1);
    LL_I2C_EnableIT_NACK(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
    LL_I2C_Enable(I2C1);

    NVIC_SetPriority(I2C1_EV_IRQn, 1);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_ER_IRQn, 1);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/*
 * Reads bytes from the internal register buffer.
 *
 * addr is the first register address to read.
 * data points to the destination buffer.
 * len is the requested number of bytes.
 *
 * The function returns the number of bytes copied. It returns -EINVAL if
 * the destination pointer is null or the start address is outside the
 * register map. If the requested range extends past REG_COUNT, the read
 * length is clipped to the valid range.
 */
int read_reg(uint32_t addr, uint8_t *data, size_t len)
{
    if (data == NULL)
    {
        return -EINVAL;
    }

    if (addr >= REG_COUNT)
    {
        return -EINVAL;
    }

    if (addr + len > REG_COUNT)
    {
        len = REG_COUNT - addr;
    }

    memcpy(data, &reg[addr], len);

    return (int)len;
}

/*
 * Writes bytes to the internal register buffer.
 *
 * addr is the first register address to write.
 * data points to the source buffer.
 * len is the requested number of bytes.
 *
 * The function returns the number of bytes copied. It returns -EINVAL if
 * the source pointer is null or the start address is outside the register
 * map. If the requested range extends past REG_COUNT, the write length is
 * clipped to the valid range.
 */
int write_reg(uint32_t addr, const uint8_t *data, size_t len)
{
    if (data == NULL)
    {
        return -EINVAL;
    }

    if (addr >= REG_COUNT)
    {
        return -EINVAL;
    }

    if (addr + len > REG_COUNT)
    {
        len = REG_COUNT - addr;
    }

    memcpy(&reg[addr], data, len);

    return (int)len;
}

/*
 * Clears the temporary I2C receive length.
 *
 * This is called when a new I2C slave receive transaction starts.
 */
void i2c_ll_reset_rx(void)
{
    i2c_rx_len = 0;
}

/*
 * Appends one byte to the temporary I2C receive buffer.
 *
 * Extra bytes are ignored when the temporary buffer is already full.
 */
void i2c_ll_rx_byte(uint8_t data)
{
    if (i2c_rx_len < I2C_RX_BUF_SIZE)
    {
        i2c_rx_buf[i2c_rx_len] = data;
        i2c_rx_len++;
    }
    else
    {
        /*
         * Ignore bytes beyond the receive buffer size.
         */
    }
}

/*
 * Handles the end of an I2C slave write transaction.
 *
 * The first received byte is treated as the start register address. The
 * remaining bytes are committed to the internal register buffer through
 * write_reg() so that register bounds are still checked.
 */
void i2c_ll_stop_detected(void)
{
    if (i2c_rx_len >= 2)
    {
        uint8_t start_addr = i2c_rx_buf[0];
        uint16_t data_len = i2c_rx_len - 1;

        /*
         * Use write_reg() instead of writing to reg[] directly so that
         * register bounds are checked consistently.
         */
        write_reg(start_addr, &i2c_rx_buf[1], data_len);
    }

    i2c_rx_len = 0;
}
