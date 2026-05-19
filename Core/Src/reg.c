#include "reg.h"
#include "main.h"
#include <string.h>
#include <errno.h>

#define I2C_RX_BUF_SIZE (REG_COUNT + 1)

static uint8_t reg[REG_COUNT] = {0};
static uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE] = {0};
static uint16_t i2c_rx_len = 0;

void reg_init(void)
{
    memset((void *)reg, 0, sizeof(reg));
    memset((void *)i2c_rx_buf, 0, sizeof(i2c_rx_buf));
    i2c_rx_len = 0;

    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
    LL_I2C_EnableIT_EVT(I2C1);
    LL_I2C_EnableIT_BUF(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
    LL_I2C_Enable(I2C1);
}

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

void I2C_LL_ResetRx(void)
{
    i2c_rx_len = 0;
}

void I2C_LL_RxByte(uint8_t data)
{
    if (i2c_rx_len < I2C_RX_BUF_SIZE)
    {
        i2c_rx_buf[i2c_rx_len] = data;
        i2c_rx_len++;
    }
    else
    {
        /*
         * 超过缓冲区的数据直接丢弃。
         * 后续可以加 debug_i2c_overflow_count++。
         */
    }
}

void I2C_LL_StopDetected(void)
{
    if (i2c_rx_len >= 2)
    {
        uint8_t start_addr = i2c_rx_buf[0];
        uint16_t data_len = i2c_rx_len - 1;

        /*
         * 不要在这里直接 memcpy 到 reg[]。
         * 必须通过 write_reg()，这样才有边界检查。
         */
        write_reg(start_addr, &i2c_rx_buf[1], data_len);
    }

    i2c_rx_len = 0;
}
