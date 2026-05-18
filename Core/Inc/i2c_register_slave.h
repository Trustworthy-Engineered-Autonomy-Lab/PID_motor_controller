#ifndef I2C_REGISTER_SLAVE_H
#define I2C_REGISTER_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * I2C 接收缓冲区大小。
 *
 * 上位机可以发送变长命令，但 MCU 内部需要限制最大接收长度。
 * 超过该长度的字节会被读取但丢弃，避免 I2C 总线卡死。
 */
#define I2C_RX_BUF_SIZE     32

/*
 * I2C 接收缓冲区。
 *
 * i2c_rx_buf[0] = 起始寄存器地址
 * i2c_rx_buf[1] = 写入到起始寄存器的数据
 * i2c_rx_buf[2] = 写入到起始寄存器 + 1 的数据
 * ...
 */
extern volatile uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE];

/*
 * 当前正在接收的一帧数据长度。
 */
extern volatile uint16_t i2c_rx_len;

/*
 * I2C 调试变量。
 */
extern volatile uint8_t debug_i2c_start_reg;
extern volatile uint16_t debug_i2c_rx_len;
extern volatile uint32_t debug_i2c_frame_count;
extern volatile uint32_t debug_i2c_overflow_count;
extern volatile uint32_t debug_i2c_discard_count;

/*
 * 接收 1 个 I2C 字节。
 *
 * 通常在 I2C1_EV_IRQHandler() 的 RXNE 分支中调用。
 */
void I2C_LL_RxByte(uint8_t data);

/*
 * I2C STOP 条件处理函数。
 *
 * 通常在 I2C1_EV_IRQHandler() 的 STOPF 分支中调用。
 * STOP 表示一帧 I2C 写入结束，此时解析整帧数据。
 */
void I2C_LL_StopDetected(void);

#ifdef __cplusplus
}
#endif

#endif
