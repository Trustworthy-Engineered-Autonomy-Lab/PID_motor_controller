#include "i2c_register_slave.h"
#include "register_map.h"

/*
 * I2C 接收缓冲区。
 */
volatile uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE] = {0};

/*
 * 当前接收帧长度。
 */
volatile uint16_t i2c_rx_len = 0;

/*
 * I2C 调试变量。
 */
volatile uint8_t debug_i2c_start_reg = 0;
volatile uint16_t debug_i2c_rx_len = 0;
volatile uint32_t debug_i2c_frame_count = 0;
volatile uint32_t debug_i2c_overflow_count = 0;
volatile uint32_t debug_i2c_discard_count = 0;

/*
 * LL I2C 字节接收函数。
 *
 * 该函数由 I2C1_EV_IRQHandler() 在 RXNE 标志置位时调用。
 *
 * 工作逻辑：
 *   1. 如果 i2c_rx_buf[] 未满，则保存该字节；
 *   2. i2c_rx_len 自增；
 *   3. 如果缓冲区已满，则该字节已从 I2C 数据寄存器读出，
 *      但不保存，只增加 debug_i2c_overflow_count。
 *
 * 注意：
 *   必须读取 I2C 数据寄存器，否则 RXNE 不会清除，
 *   I2C 总线可能卡住。
 */
void I2C_LL_RxByte(uint8_t data)
{
    if (i2c_rx_len < I2C_RX_BUF_SIZE)
    {
        i2c_rx_buf[i2c_rx_len] = data;
        i2c_rx_len++;
    }
    else
    {
        debug_i2c_overflow_count++;
    }
}

/*
 * LL I2C STOP 条件处理函数。
 *
 * 当前协议格式：
 *   i2c_rx_buf[0] = 起始寄存器地址 start_reg
 *   i2c_rx_buf[1] = 写入 start_reg 的数据
 *   i2c_rx_buf[2] = 写入 start_reg + 1 的数据
 *   ...
 *
 * 示例：
 *   i2ctransfer -y 7 w4@0x08 0x01 0x01 0x0E 0x06
 *
 * 解析结果：
 *   reg_map[0x01] = 0x01
 *   reg_map[0x02] = 0x0E
 *   reg_map[0x03] = 0x06
 *
 * 然后调用 Register_Map_Apply() 应用到：
 *   motor_mode
 *   target_pwm_us
 *   target_rpm
 */
void I2C_LL_StopDetected(void)
{
    debug_i2c_rx_len = i2c_rx_len;
    debug_i2c_frame_count++;

    /*
     * 至少需要 2 个字节：
     *   第 1 字节：起始寄存器地址
     *   第 2 字节：至少 1 个数据
     */
    if (i2c_rx_len >= 2)
    {
        uint8_t start_reg = i2c_rx_buf[0];
        debug_i2c_start_reg = start_reg;

        for (uint16_t i = 1; i < i2c_rx_len; i++)
        {
            uint16_t reg_addr = (uint16_t)start_reg + (i - 1);

            if (reg_addr < REG_MAP_SIZE)
            {
                reg_map[reg_addr] = i2c_rx_buf[i];
            }
            else
            {
                /*
                 * 超出软件寄存器范围，直接舍弃。
                 */
                debug_i2c_discard_count++;
            }
        }

        Register_Map_Apply();
    }

    /*
     * 清空接收长度，准备下一帧。
     * 不需要清空整个 i2c_rx_buf[]，下一帧会覆盖旧数据。
     */
    i2c_rx_len = 0;
}
