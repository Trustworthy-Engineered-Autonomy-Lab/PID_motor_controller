/*
 * 当前版本说明：
 *
 * 1. 本文件实现 STM32 端电机控制逻辑，当前采用“I2C 变长寄存器控制模式”。
 *    Jetson / 上位机通过 I2C 向 STM32 写入一帧变长命令：
 *
 *      第 1 字节：起始寄存器地址 start_reg
 *      后续字节：从 start_reg 开始，按地址递增依次写入的软件寄存器数据
 *
 *    例如：
 *      i2ctransfer -y 7 w4@0x08 0x01 0x01 0x0E 0x06
 *
 *    含义为：
 *      从寄存器 0x01 开始连续写入 3 个数据：
 *        reg_map[0x01] = 0x01  -> 开环 PWM 模式
 *        reg_map[0x02] = 0x0E  -> PWM 脉宽低字节
 *        reg_map[0x03] = 0x06  -> PWM 脉宽高字节
 *
 *      最终 target_pwm_us = 0x060E = 1550 us。
 *
 * 2. STM32 的 I2C1 当前使用 LL 库接收数据。
 *    I2C 字节接收不再使用 HAL_I2C_Slave_Receive_IT()，
 *    而是在 I2C1_EV_IRQHandler() 中逐字节读取 RXNE 数据。
 *
 *    接收流程：
 *      ADDR 中断：主机寻址到 STM32，清空 i2c_rx_len，准备接收新一帧
 *      RXNE 中断：每收到 1 字节，调用 I2C_LL_RxByte() 保存到 i2c_rx_buf[]
 *      STOP 中断：主机结束本次传输，调用 I2C_LL_StopDetected() 解析整帧数据
 *
 * 3. 当前软件寄存器表 reg_map[]：
 *      REG_MODE         = 0x01：控制模式寄存器
 *      REG_PWM_US_L     = 0x02：开环 PWM 脉宽低字节
 *      REG_PWM_US_H     = 0x03：开环 PWM 脉宽高字节
 *      REG_TARGET_RPM_L = 0x04：目标 RPM 低字节
 *      REG_TARGET_RPM_H = 0x05：目标 RPM 高字节
 *
 *    如果上位机连续写入的数据超过 REG_MAP_SIZE 定义的寄存器范围，
 *    超出的数据会被读取但舍弃，避免越界写入或 I2C 总线卡死。
 *
 * 4. 当前支持 3 种控制模式：
 *      MOTOR_MODE_STOP         = 0：停止模式，强制输出中位 PWM，通常为 1500 us
 *      MOTOR_MODE_OPENLOOP_PWM = 1：开环 PWM 模式，直接输出 target_pwm_us
 *      MOTOR_MODE_PID_RPM      = 2：PID RPM 模式，根据 target_rpm 和 motor_rpm 做闭环控制
 *
 * 5. 当前主要稳定使用的是 MOTOR_MODE_OPENLOOP_PWM。
 *    MOTOR_MODE_PID_RPM 已经接入 pid_pwm_update()，但 PID 参数和低速测速稳定性
 *    仍需要后续继续调试。
 *
 * 6. TIM1 周期性置位 pid_update_flag。
 *    虽然变量名仍叫 pid_update_flag，但在当前版本中它更准确地表示
 *    “控制循环刷新标志”：开环模式下用于周期性刷新 PWM，PID 模式下用于周期性执行 PID。
 */

/*
 * STM32CubeMonitor 建议监控变量：
 *
 * 1. I2C 接收与解析相关：
 *      i2c_rx_len
 *      debug_i2c_rx_len
 *      debug_i2c_start_reg
 *      debug_i2c_frame_count
 *      debug_i2c_overflow_count
 *      debug_i2c_discard_count
 *
 *    含义：
 *      i2c_rx_len：当前正在接收的一帧数据长度
 *      debug_i2c_rx_len：最近一帧完整 I2C 命令的长度
 *      debug_i2c_start_reg：最近一帧命令的起始寄存器地址
 *      debug_i2c_frame_count：成功收到 STOP 并进入解析流程的帧计数
 *      debug_i2c_overflow_count：接收缓冲区 i2c_rx_buf[] 已满后丢弃的字节数
 *      debug_i2c_discard_count：寄存器地址超出 reg_map[] 范围后丢弃的数据数
 *
 * 2. 软件寄存器表相关：
 *      reg_map[1]：控制模式
 *      reg_map[2]：PWM 脉宽低字节
 *      reg_map[3]：PWM 脉宽高字节
 *      reg_map[4]：目标 RPM 低字节
 *      reg_map[5]：目标 RPM 高字节
 *
 * 3. 控制状态相关：
 *      motor_mode
 *      debug_motor_mode_view
 *
 *    motor_mode 是程序实际使用的控制模式：
 *      0 = 停止模式
 *      1 = 开环 PWM 模式
 *      2 = PID RPM 模式
 *
 *    debug_motor_mode_view 是为了在 CubeMonitor 中更直观地观察状态：
 *      1000 = 停止模式
 *      2000 = 开环 PWM 模式
 *      3000 = PID RPM 模式
 *      9999 = 异常模式
 *
 * 4. PWM 输出相关：
 *      target_pwm_us
 *      debug_pwm_us
 *      debug_pwm_ccr
 *
 *    target_pwm_us：上位机通过寄存器写入的目标 PWM 脉宽，单位 us
 *    debug_pwm_us：当前实际输出到 ESC 的 PWM 脉宽，单位 us
 *    debug_pwm_ccr：当前实际写入 TIM2 CCR 的比较值
 *
 * 5. 测速与 PID 相关：
 *      hall_capture_value
 *      motor_rpm
 *      target_rpm
 *
 *    motor_rpm 当前在开环模式下主要用于观察；
 *    在 PID RPM 模式下，motor_rpm 会作为 PID 闭环反馈值。
 *
 * 推荐最小监控组合：
 *      debug_i2c_frame_count
 *      debug_i2c_rx_len
 *      debug_i2c_start_reg
 *      reg_map[1]
 *      reg_map[2]
 *      reg_map[3]
 *      motor_mode
 *      debug_motor_mode_view
 *      target_pwm_us
 *      debug_pwm_us
 *      debug_pwm_ccr
 *
 * 判断链路：
 *      上位机发送 i2ctransfer
 *          -> debug_i2c_frame_count 增加
 *          -> reg_map[] 正确变化
 *          -> motor_mode / target_pwm_us 正确变化
 *          -> debug_pwm_us / debug_pwm_ccr 正确变化
 *          -> 电机实际状态变化
 */

/* Includes */
#include "user.h"
/* End Includes */


/* Variable Declarations */

/*
 * 外部外设句柄：
 * 这些变量由 CubeMX 在 main.c / tim.c / i2c.c 中生成，
 * user.c 通过 extern 引用它们，以便启动 PWM、I2C、定时器和霍尔测速。
 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/*
 * 兼容旧版调试用变量。
 *
 * 早期版本中，上位机直接向 speed_setpoint 写入 PWM 脉宽。
 * 当前版本已经改为寄存器协议，真正的开环 PWM 目标值是 target_pwm_us。
 *
 * 为了方便继续在 STM32CubeMonitor 中观察旧变量，
 * 当 REG_PWM_US 被写入时，会同步执行：
 *      speed_setpoint = target_pwm_us;
 *
 * 注意：当前控制逻辑不再直接依赖 speed_setpoint。
 */
volatile int16_t speed_setpoint = 0;

/*
 * I2C 软件寄存器表设计：
 *
 * 上位机现在发送变长命令：
 *   第 1 字节：起始寄存器地址
 *   后续字节：从该地址开始依次写入的数据
 *
 * 例如：
 *   i2ctransfer -y 7 w4@0x08 0x01 0x01 0x0E 0x06
 *
 * 表示：
 *   reg_map[0x01] = 0x01  -> 开环 PWM 模式
 *   reg_map[0x02] = 0x0E  -> PWM 低字节
 *   reg_map[0x03] = 0x06  -> PWM 高字节
 *
 * 最终：
 *   target_pwm_us = 0x060E = 1550 us
 */
#define REG_MODE            0x01

#define REG_PWM_US_L        0x02
#define REG_PWM_US_H        0x03

#define REG_TARGET_RPM_L    0x04
#define REG_TARGET_RPM_H    0x05

/*
 * 软件寄存器表大小。
 * 当前有效寄存器地址为 0x01 ~ 0x05。
 * 数组下标 0 暂时不用，所以大小设为 0x06，可访问 reg_map[0] ~ reg_map[5]。
 */
#define REG_MAP_SIZE        0x06

/*
 * I2C 接收缓冲区大小。
 * 协议逻辑上支持变长写入，但 MCU 内部必须有一个实际缓冲区上限。
 * 超出该缓冲区的字节会被读取但丢弃，防止 I2C 卡死。
 */
#define I2C_RX_BUF_SIZE     32

/*
 * 控制模式定义
 */
#define MOTOR_MODE_STOP          0
#define MOTOR_MODE_OPENLOOP_PWM  1
#define MOTOR_MODE_PID_RPM       2

/*
 * I2C 接收缓冲区：
 * i2c_rx_buf[0] = 起始寄存器地址
 * i2c_rx_buf[1] = 写入到起始寄存器的数据
 * i2c_rx_buf[2] = 写入到起始寄存器 + 1 的数据
 * ...
 */
volatile uint8_t i2c_rx_buf[I2C_RX_BUF_SIZE] = {0};
volatile uint16_t i2c_rx_len = 0;

/*
 * 软件寄存器镜像表。
 * 上位机写入的数据先进入 reg_map[]，
 * 再由 Register_Map_Apply() 转换为实际控制变量。
 */
volatile uint8_t reg_map[REG_MAP_SIZE] = {0};

/*
 * 当前电机控制模式。
 */
volatile uint8_t motor_mode = MOTOR_MODE_STOP;

/*
 * 给 STM32CubeMonitor 观察用的模式显示变量。
 * 1000 = 停止模式
 * 2000 = 开环 PWM 模式
 * 3000 = PID RPM 模式
 * 9999 = 异常模式
 */
volatile uint16_t debug_motor_mode_view = 1000;

/*
 * 开环 PWM 目标脉宽，单位 us。
 */
volatile int16_t target_pwm_us = PWM_US_NEUTRAL;

/*
 * PID 目标转速，单位 RPM。
 */
volatile int16_t target_rpm = 0;

/*
 * I2C 调试变量，方便在 CubeMonitor 中判断是否收到完整命令。
 */
volatile uint8_t debug_i2c_start_reg = 0;
volatile uint16_t debug_i2c_rx_len = 0;
volatile uint32_t debug_i2c_frame_count = 0;
volatile uint32_t debug_i2c_overflow_count = 0;
volatile uint32_t debug_i2c_discard_count = 0;

/* 标志位：由中断置 1，由主循环清 0。 */
volatile uint8_t pwm_update_flag = 0;	// 当前版本暂未使用，保留给后续 PWM 更新逻辑
volatile uint8_t pid_update_flag = 0;	// TIM1 周期触发；当前版本用作 PWM 周期刷新触发
volatile uint8_t speed_update_flag = 0;	// I2C 收到新指令后置 1
volatile uint8_t hall_update_flag = 0;	// 霍尔输入捕获或 TIM3 溢出后置 1

/* 调试变量：方便用 STM32CubeMonitor / 调试器观察当前输出。 */
volatile int16_t debug_pwm_us = PWM_US_NEUTRAL;	// 当前实际输出的 PWM 脉宽，单位 us
volatile uint32_t debug_pwm_ccr = 0;			// 当前实际写入 TIM2 CCR 的值

/*
 * 电机实际转速，由 TIM3 霍尔传感器输入捕获更新。
 *
 * 在 MOTOR_MODE_OPENLOOP_PWM 模式下，motor_rpm 只用于观察，不参与 PWM 计算。
 * 在 MOTOR_MODE_PID_RPM 模式下，motor_rpm 会作为 PID 闭环反馈值使用。
 */
static volatile float motor_rpm = 0;

/*
 * PID 闭环模式下的目标转速。
 *
 * 当前主要使用开环 PWM 模式，因此 rpm_setpoint 暂时不是主要控制变量。
 * 当 REG_TARGET_RPM 被写入时，会同步更新 rpm_setpoint，
 * 后续调试 MOTOR_MODE_PID_RPM 时可作为目标转速使用。
 */
static volatile float rpm_setpoint = 0;

/* 霍尔输入捕获值：记录相邻霍尔边沿之间的定时器计数。 */
volatile uint32_t hall_capture_value = 0;

/*
 * 记录上一次霍尔传感器更新时间。
 * 当前代码中暂未实际使用，可用于后续判断“长时间无霍尔信号 -> 电机停止”。
 */
volatile uint32_t lastHallSensorUpdate = 0;

/* 电机 PID 控制器实例。当前开环 PWM 测试模式下暂不参与输出。 */
PID_t motor_pid;

/* PID 参数：当前保留，供后续闭环转速控制使用。 */
const float dt = TIM1_PER_MS / 1000; 							// PID 更新周期，单位 s
const float Kp = 0.00001; 										// 比例系数
const float Ki = 0.000005; 										// 积分系数
const float Kd = 0; 											// 微分系数
const float Integral_max = 100000.0f; 							// 积分限幅，防止积分饱和
const float pid_max = PWM_MAX_PULSEWIDTH - PWM_ZERO_PULSEWIDTH;	// PID 输出最大脉宽修正量
/* End Variable Definitions */


/* Function Declarations */
void TIM_PER_CHECK(void);
void Register_Map_Init(void);
void Register_Map_Apply(void);
void Update_Debug_Motor_Mode_View(void);
void I2C_LL_RxByte(uint8_t data);
void I2C_LL_StopDetected(void);

/* 速度 / 转速 / PWM 转换相关函数：当前主要保留给后续 PID 闭环版本。 */
float rpm_update(int16_t speed_setpoint);
void openloop_pwm_update(float rpm_setpoint);
void pid_pwm_update(float rpm_setpoint);
uint32_t rpm_to_pwm_duty(int16_t rpm_setpoint);
void User_Error_Handler(uint8_t count);
/* End Function Declarations */


/* Function Definitions */
uint32_t pwm_us_to_ccr(int16_t pulse_us);

/*
 * 将 PWM 脉宽 us 转换为 TIM2 CCR 值。
 *
 * 输入：pulse_us，单位 us，典型范围 1000~2000。
 * 处理：先限幅，再转换为 ms，因为 PWM_PULSEWIDTH_TO_CCR() 原本接收的是 ms。
 * 输出：可以直接写入 TIM2 CCR 的比较值。
 *
 * 注意：当前 User_Loop() 实际使用的是宏 PWM_US_TO_CCR()，
 * 这个函数可以作为等价的函数版本保留，方便调试或后续替换宏。
 */
uint32_t pwm_us_to_ccr(int16_t pulse_us)
{
    if (pulse_us < 1000) {
        pulse_us = 1000;
    } else if (pulse_us > 2000) {
        pulse_us = 2000;
    }

    /*
     * PWM_PULSEWIDTH_TO_CCR() 原来接收的是 ms，
     * 所以 1500 us 要先变成 1.5 ms。
     */
    float pulse_ms = (float)pulse_us / 1000.0f;

    return (uint32_t)PWM_PULSEWIDTH_TO_CCR(pulse_ms);
}

/*
 * 用户初始化函数。
 *
 * main.c 在完成 CubeMX 自动生成的外设初始化后，会调用 User_Init()。
 * 本函数负责启动本控制逻辑需要的软件状态和外设功能。
 *
 * 初始化内容：
 * 1. 检查 TIM1 / TIM2 / TIM3 的周期是否符合 user.h 中的预期配置；
 * 2. 初始化软件寄存器表 reg_map[]，默认进入停止模式，PWM = 1500 us；
 * 3. 启动 LL I2C 从机接收相关中断；
 * 4. 启动 TIM2 CH2 PWM 输出，并先输出默认中位 PWM；
 * 5. 启动 TIM3 霍尔传感器输入捕获和溢出中断；
 * 6. 启动 TIM1 控制周期中断；
 * 7. 初始化 PID 控制器参数。
 *
 * 注意：
 * 当前 I2C1 已经改为 LL 库模式，因此这里不再调用
 * HAL_I2C_Slave_Receive_IT()。
 */
void User_Init(void)
{
    TIM_PER_CHECK();

    Register_Map_Init();

    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
    LL_I2C_EnableIT_EVT(I2C1);
    LL_I2C_EnableIT_BUF(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
    LL_I2C_Enable(I2C1);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_CCR_DEFAULT);

    HAL_TIMEx_HallSensor_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim1);

    PID_Init(&motor_pid, Kp, Ki, Kd, Integral_max, pid_max);
}

/*
 * 初始化软件寄存器表 reg_map[] 的默认值。
 *
 * 上电后为了安全，默认设置为：
 *   REG_MODE = MOTOR_MODE_STOP
 *   REG_PWM_US_L / REG_PWM_US_H = PWM_US_NEUTRAL，即 1500 us
 *   REG_TARGET_RPM_L / REG_TARGET_RPM_H = 0
 *
 * 初始化完成后调用 Register_Map_Apply()，
 * 将 reg_map[] 中的默认值同步到实际控制变量：
 *   motor_mode
 *   target_pwm_us
 *   target_rpm
 *   speed_setpoint
 *   debug_motor_mode_view
 */
void Register_Map_Init(void)
{
    for (uint8_t i = 0; i < REG_MAP_SIZE; i++) {
        reg_map[i] = 0;
    }

    reg_map[REG_MODE] = MOTOR_MODE_STOP;

    reg_map[REG_PWM_US_L] = (uint8_t)(PWM_US_NEUTRAL & 0xFF);
    reg_map[REG_PWM_US_H] = (uint8_t)((PWM_US_NEUTRAL >> 8) & 0xFF);

    reg_map[REG_TARGET_RPM_L] = 0;
    reg_map[REG_TARGET_RPM_H] = 0;

    Register_Map_Apply();
}

/*
 * 根据 motor_mode 更新 CubeMonitor 显示变量。
 */
void Update_Debug_Motor_Mode_View(void)
{
    switch (motor_mode)
    {
        case MOTOR_MODE_STOP:
            debug_motor_mode_view = 1000;
            break;

        case MOTOR_MODE_OPENLOOP_PWM:
            debug_motor_mode_view = 2000;
            break;

        case MOTOR_MODE_PID_RPM:
            debug_motor_mode_view = 3000;
            break;

        default:
            debug_motor_mode_view = 9999;
            break;
    }
}

/*
 * 将软件寄存器表 reg_map[] 中的原始字节数据，
 * 转换并同步到实际参与控制的变量。
 *
 * I2C 接收阶段只负责把数据写入 reg_map[]，
 * 不直接修改 PWM 输出或 PID 目标。
 * 这样可以把“通信协议解析”和“控制变量更新”分开，逻辑更清晰。
 *
 * 本函数主要做 3 件事：
 *
 * 1. 应用控制模式：
 *      reg_map[REG_MODE] -> motor_mode
 *
 *    只接受合法模式：
 *      0 = MOTOR_MODE_STOP
 *      1 = MOTOR_MODE_OPENLOOP_PWM
 *      2 = MOTOR_MODE_PID_RPM
 *
 *    如果收到非法模式值，则强制回到 MOTOR_MODE_STOP。
 *
 * 2. 应用开环 PWM 脉宽：
 *      reg_map[REG_PWM_US_L] 和 reg_map[REG_PWM_US_H]
 *      组合成 16 位 target_pwm_us。
 *
 *    组合方式：
 *      target_pwm_us = low_byte | (high_byte << 8)
 *
 *    随后使用 PWM_US_CLAMP() 限幅到 1000~2000 us。
 *
 * 3. 应用目标 RPM：
 *      reg_map[REG_TARGET_RPM_L] 和 reg_map[REG_TARGET_RPM_H]
 *      组合成 16 位 target_rpm。
 *
 *    target_rpm 主要用于 MOTOR_MODE_PID_RPM 模式。
 *
 * 另外：
 *   speed_setpoint 是旧版调试变量，会同步为 target_pwm_us，
 *   方便继续在 CubeMonitor 中观察。
 */
void Register_Map_Apply(void)
{
    /*
     * 1. 应用控制模式
     */
    uint8_t mode = reg_map[REG_MODE];

    if (mode == MOTOR_MODE_STOP ||
        mode == MOTOR_MODE_OPENLOOP_PWM ||
        mode == MOTOR_MODE_PID_RPM)
    {
        motor_mode = mode;
    }
    else
    {
        motor_mode = MOTOR_MODE_STOP;
        reg_map[REG_MODE] = MOTOR_MODE_STOP;
    }

    Update_Debug_Motor_Mode_View();

    /*
     * 2. 应用 PWM 脉宽
     *    低字节在 REG_PWM_US_L，高字节在 REG_PWM_US_H。
     */
    int16_t pwm_value = (int16_t)((uint16_t)reg_map[REG_PWM_US_L] |
                                 ((uint16_t)reg_map[REG_PWM_US_H] << 8));

    target_pwm_us = PWM_US_CLAMP(pwm_value);

    /*
     * 同步旧变量，方便继续在 CubeMonitor 中观察 speed_setpoint。
     */
    speed_setpoint = target_pwm_us;

    /*
     * 如果输入 PWM 超出范围，reg_map 也同步修正为限幅后的值，
     * 这样 CubeMonitor 看到的寄存器表和实际控制值一致。
     */
    reg_map[REG_PWM_US_L] = (uint8_t)(target_pwm_us & 0xFF);
    reg_map[REG_PWM_US_H] = (uint8_t)(((uint16_t)target_pwm_us >> 8) & 0xFF);

    /*
     * 3. 应用目标 RPM。
     */
    target_rpm = (int16_t)((uint16_t)reg_map[REG_TARGET_RPM_L] |
                          ((uint16_t)reg_map[REG_TARGET_RPM_H] << 8));

    rpm_setpoint = (float)target_rpm;

    speed_update_flag = 1;
}

/*
 * LL I2C 字节接收函数。
 *
 * 该函数由 I2C1_EV_IRQHandler() 在 RXNE 标志置位时调用。
 * RXNE 表示 I2C1 数据寄存器中已经收到 1 个新字节。
 *
 * 工作逻辑：
 *   1. 如果 i2c_rx_buf[] 还没有满，则把该字节保存到缓冲区；
 *   2. i2c_rx_len 自增，记录当前帧已经收到的字节数；
 *   3. 如果缓冲区已满，则该字节虽然已经从 I2C 数据寄存器读出，
 *      但不再保存，只增加 debug_i2c_overflow_count。
 *
 * 注意：
 * 必须在中断里读取 I2C 数据寄存器，否则 RXNE 不会清除，
 * I2C 总线可能卡住，导致上位机出现 connection timed out。
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
 * 该函数由 I2C1_EV_IRQHandler() 在检测到 STOP 标志时调用。
 * STOP 表示上位机已经结束本次 I2C 写入，因此此时 i2c_rx_buf[]
 * 中保存的是一帧完整命令。
 *
 * 当前协议格式：
 *   i2c_rx_buf[0] = 起始寄存器地址 start_reg
 *   i2c_rx_buf[1] = 写入 start_reg 的数据
 *   i2c_rx_buf[2] = 写入 start_reg + 1 的数据
 *   i2c_rx_buf[3] = 写入 start_reg + 2 的数据
 *   ...
 *
 * 示例：
 *   上位机发送：
 *      i2ctransfer -y 7 w4@0x08 0x01 0x01 0x0E 0x06
 *
 *   STM32 收到：
 *      i2c_rx_buf[0] = 0x01
 *      i2c_rx_buf[1] = 0x01
 *      i2c_rx_buf[2] = 0x0E
 *      i2c_rx_buf[3] = 0x06
 *
 *   解析结果：
 *      reg_map[0x01] = 0x01
 *      reg_map[0x02] = 0x0E
 *      reg_map[0x03] = 0x06
 *
 *   最终：
 *      motor_mode = MOTOR_MODE_OPENLOOP_PWM
 *      target_pwm_us = 0x060E = 1550 us
 *
 * 安全处理：
 *   如果 start_reg + 数据偏移 超过 REG_MAP_SIZE，
 *   超出的数据不会写入 reg_map[]，只增加 debug_i2c_discard_count。
 *
 * 调试信息：
 *   debug_i2c_rx_len 记录最近一帧的长度；
 *   debug_i2c_frame_count 记录收到完整帧的次数；
 *   debug_i2c_start_reg 记录最近一帧的起始寄存器地址。
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
                 * 超出预设寄存器数量，直接舍弃。
                 */
                debug_i2c_discard_count++;
            }
        }

        Register_Map_Apply();
    }

    /*
     * 清空接收长度，准备下一帧。
     * 不一定要清空整个 i2c_rx_buf，因为下一次会覆盖。
     */
    i2c_rx_len = 0;
}

/*
 * 用户主循环函数。
 *
 * main.c 的 while(1) 中会不断调用 User_Loop()。
 * 本函数不直接阻塞等待，而是通过中断置位的标志位来决定是否执行控制更新。
 *
 * 当前主要使用 3 个标志：
 *   speed_update_flag：I2C 写入寄存器后置位，目前主要作为“收到新控制命令”的提示
 *   hall_update_flag：TIM3 霍尔输入捕获或超时后置位
 *   pid_update_flag：TIM1 周期中断置位，表示到达一个控制刷新周期
 *
 * 控制逻辑：
 *   每当 pid_update_flag 被 TIM1 置位后，根据 motor_mode 选择控制模式：
 *
 *   1. MOTOR_MODE_STOP：
 *        强制输出 PWM_US_NEUTRAL，通常为 1500 us，用于停止电机。
 *
 *   2. MOTOR_MODE_OPENLOOP_PWM：
 *        直接输出 target_pwm_us 对应的 PWM。
 *        这是当前主要稳定使用的模式。
 *
 *   3. MOTOR_MODE_PID_RPM：
 *        调用 pid_pwm_update(target_rpm)，根据 target_rpm 和 motor_rpm 做闭环控制。
 *        当前该模式已接入，但仍需要继续调参验证。
 *
 *   4. default：
 *        如果 motor_mode 出现非法值，强制切回停止模式，保证安全。
 */
void User_Loop(void)
{
    if (speed_update_flag == 1) {
        speed_update_flag = 0;
    }

    if (hall_update_flag == 1) {
        hall_update_flag = 0;
    }

    if (pid_update_flag == 1) {
        pid_update_flag = 0;

        switch (motor_mode)
        {
            case MOTOR_MODE_STOP:
            {
                int16_t pulse_us = PWM_US_NEUTRAL;
                uint32_t pwm_ccr = PWM_US_TO_CCR(pulse_us);

                debug_pwm_us = pulse_us;
                debug_pwm_ccr = pwm_ccr;

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_ccr);
                break;
            }

            case MOTOR_MODE_OPENLOOP_PWM:
            {
                int16_t pulse_us = PWM_US_CLAMP(target_pwm_us);
                uint32_t pwm_ccr = PWM_US_TO_CCR(pulse_us);

                debug_pwm_us = pulse_us;
                debug_pwm_ccr = pwm_ccr;

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_ccr);
                break;
            }

            case MOTOR_MODE_PID_RPM:
            {
                pid_pwm_update((float)target_rpm);
                break;
            }

            default:
            {
                motor_mode = MOTOR_MODE_STOP;
                Update_Debug_Motor_Mode_View();

                int16_t pulse_us = PWM_US_NEUTRAL;
                uint32_t pwm_ccr = PWM_US_TO_CCR(pulse_us);

                debug_pwm_us = pulse_us;
                debug_pwm_ccr = pwm_ccr;

                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_ccr);
                break;
            }
        }
    }
}

/*
 * 检查定时器周期是否符合预期。
 * 如果 CubeMX 中预分频 PSC、自动重装载 ARR 或时钟源配置错误，
 * 这里会进入 User_Error_Handler()，通过 LED 闪烁次数提示错误来源。
 */
void TIM_PER_CHECK(void){
	/* TIM1：控制循环周期检查，允许误差 0.1 ms。 */
	if(fabs(TIM1_PER_MS - EXPECTED_TIM1_PER_MS) > .1){
		User_Error_Handler(1);
	}

	/* TIM2：PWM 输出周期检查，允许误差 0.1 ms。 */
	if(fabs(TIM2_PER_MS - EXPECTED_TIM2_PER_MS) > .1){
		User_Error_Handler(2);
	}

	/* TIM3：霍尔测速定时器周期检查，允许误差 1 ms。 */
	if(fabs(TIM3_PER_MS - EXPECTED_TIM3_PER_MS) > 1){
		User_Error_Handler(3);
	}


}


/*
 * 将上位机输入的速度类指令转换为目标 RPM。
 *
 * 当前版本中，目标 RPM 主要通过软件寄存器：
 *   REG_TARGET_RPM_L
 *   REG_TARGET_RPM_H
 * 写入，并在 Register_Map_Apply() 中组合成 target_rpm。
 *
 * 因此该函数目前不是主控制路径的一部分，只作为旧版接口保留。
 * 后续如果需要支持更复杂的速度单位转换，可以在这里扩展，例如：
 *   线速度 -> 轮速 RPM
 *   油门百分比 -> 目标 RPM
 *   自定义速度单位 -> 目标 RPM
 */
float rpm_update(int16_t speed_setpoint){
	return (float)speed_setpoint;
}


/*
 * 旧版开环 PWM 更新函数，当前主控制路径未调用。
 *
 * 早期版本中，该函数直接把输入值当作 TIM2 CCR 值写入 PWM 输出。
 * 当前版本已经改为：
 *   上位机写 REG_PWM_US_L / REG_PWM_US_H
 *      -> Register_Map_Apply() 得到 target_pwm_us
 *      -> User_Loop() 中转换为 CCR
 *      -> TIM2 CH2 输出 PWM
 *
 * 因此本函数主要保留用于参考或后续临时调试。
 *
 * 注意：
 * 这里的参数名 rpm_setpoint 容易误导。
 * 在该函数内部，它实际被当作 CCR 值使用，并不是真正的 RPM。
 */
void openloop_pwm_update(float rpm_setpoint) // non_pid algo
{
	uint32_t new_ccr_value;

	/* 将输入限制在允许的 CCR 范围内，避免 PWM 输出超限。 */
	if (rpm_setpoint < PWM_CCR_MIN) {
		new_ccr_value = PWM_CCR_MIN;
	} else if (rpm_setpoint > PWM_CCR_MAX) {
		new_ccr_value = PWM_CCR_MAX;
	} else {
		new_ccr_value = (uint32_t)rpm_setpoint;
	}

	/* 更新 TIM2 CH2 比较值，从而改变 PWM 脉宽。 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, new_ccr_value);

    /* 可选调试输出：如果后续配置 UART / SWO，可用于打印 RPM 和 PWM。 */
    // printf("RPM: %d, PWM: %lu\r\n", rpm_setpoint, new_pwm_value);
}

/*
 * PID 闭环 PWM 更新函数。
 *
 * 在 MOTOR_MODE_PID_RPM 模式下，User_Loop() 会调用本函数。
 *
 * 输入：
 *   rpm_setpoint：目标转速 RPM
 *
 * 反馈：
 *   motor_rpm：由 TIM3 霍尔输入捕获计算得到的实际转速
 *
 * 控制过程：
 *   1. PID_Compute() 根据目标转速和实际转速计算 motor_pid.output；
 *   2. motor_pid.output 表示相对于中位 PWM 脉宽的修正量；
 *   3. 在 PWM_ZERO_PULSEWIDTH 基础上叠加 PID 输出；
 *   4. 将脉宽转换为 TIM2 CCR；
 *   5. 写入 TIM2 CH2 输出给 ESC。
 *
 * 当前限制：
 *   该函数目前只考虑正向输出。
 *   如果 PID 计算结果低于中位 PWM，则会被限制到 PWM_CCR_DEFAULT。
 *   后续如果需要反转，需要重新设计正转 / 停止 / 反转状态机。
 */
void pid_pwm_update(float rpm_setpoint) {

	/* 计算 PID 输出。motor_pid.output 表示相对于中位脉宽的修正量。 */
	PID_Compute(&motor_pid, rpm_setpoint, motor_rpm, dt);

	/* 将 PID 输出转换为实际 PWM 脉宽，再转换为 CCR。 */
	float pulse_width = PWM_ZERO_PULSEWIDTH + motor_pid.output;
	uint32_t ccr_output = (uint32_t)PWM_PULSEWIDTH_TO_CCR(pulse_width);

	/*
	 * 当前未加入反转逻辑，所以低于中位值的输出被限制到中位值。
	 * 后续如果需要电机反转，需要重新设计正转 / 停止 / 反转状态机。
	 */
	if(ccr_output < PWM_CCR_DEFAULT) {
		ccr_output = PWM_CCR_DEFAULT;
	} else if (ccr_output > PWM_CCR_MAX) {
		ccr_output = PWM_CCR_MAX;
	}

	/* 更新 PWM 输出。 */
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_output);

}

/*
 * @brief RPM 到 PWM CCR 的映射函数，当前未实现。
 * @param rpm_setpoint 目标转速 RPM
 * @retval PWM CCR 值
 *
 * 当前函数直接 return 0，不应在实际控制中调用。
 * 后续可以通过实验记录“PWM 脉宽 / CCR 与实际 RPM”的对应关系，
 * 再用线性拟合或查表方式实现该函数。
 */
uint32_t rpm_to_pwm_duty(int16_t rpm_setpoint)
{
	/*
    uint32_t pwm_value;

    // Ensure RPM is within valid range
    if (rpm < 0) {
        rpm = 0;
    } else if (rpm > RPM_MAX_VALUE) {
        rpm = RPM_MAX_VALUE;
    }

    // Linear mapping: PWM = (RPM / RPM_MAX) * PWM_MAX
    pwm_value = ((uint32_t)rpm * PWM_MAX_DUTY) / RPM_MAX_VALUE;

    // Ensure PWM value is within bounds
    if (pwm_value > PWM_MAX_DUTY) {
        pwm_value = PWM_MAX_DUTY;
    }

    return pwm_value;
    */
	return 0;
}
/* End Function Definition */


/* Interrupt Functions */
/*
 * 定时器周期中断回调函数。
 *
 * 该函数由 HAL_TIM_IRQHandler() 间接调用。
 *
 * TIM1：
 *   作为控制循环定时器。
 *   每次 TIM1 周期到达时，置位 pid_update_flag。
 *   User_Loop() 检测到该标志后，根据 motor_mode 刷新 PWM 或执行 PID。
 *
 * TIM3：
 *   用于霍尔测速超时判断。
 *   如果 TIM3 发生溢出，但在此之前没有新的霍尔输入捕获，
 *   说明较长时间内没有检测到霍尔边沿，可以认为电机转速为 0。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        pid_update_flag = 1;
    }

    if (htim->Instance == TIM3) {
    	if (hall_update_flag == 0){
    		/* 没有输入捕获却发生 TIM3 溢出，说明长时间没有霍尔边沿，认为电机停止。 */
    		motor_rpm = 0;
    		hall_update_flag = 1;
    	}

    }

}


/*
 * TIM 输入捕获回调函数，用于霍尔传感器测速。
 *
 * 当 TIM3 捕获到霍尔传感器边沿时，HAL 会调用本函数。
 *
 * hall_capture_value：
 *   保存相邻霍尔边沿之间的定时器计数值。
 *
 * motor_rpm：
 *   根据 hall_capture_value 和 HALL_CAPTURE_TO_RPM() 宏换算得到。
 *
 * 注意：
 *   电机刚从停止进入转动时，第一次捕获值可能异常。
 *   当前代码在 motor_rpm 为 0 时，先赋值 MIN_MOTOR_RPM，
 *   避免第一次捕获直接产生异常 RPM。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	hall_update_flag = 1;

	if (htim->Instance == TIM3) {
		 /* 读取 TIM3 CCR1，获得本次霍尔边沿对应的捕获值。 */
        hall_capture_value = htim->Instance->CCR1; // Compare/Capture Register 1 store # of ticks between transitions

        /*
         * 如果之前已经有有效转速，则根据本次捕获值计算 RPM。
         * 如果 motor_rpm 之前为 0，则先给一个最小转速，避免从停止状态第一次捕获时直接计算异常值。
         */
        if (motor_rpm > 0) {
        	motor_rpm = HALL_CAPTURE_TO_RPM(hall_capture_value);
        }
        else
        {
        	motor_rpm = MIN_MOTOR_RPM;
        }
    }
}
/* End Interrupt Functions */

/*
 * 自定义错误处理函数。
 *
 * 用途：当定时器周期检查失败时，通过绿色 LED 闪烁次数提示错误来源。
 * 1 次闪烁：TIM1 周期错误
 * 2 次闪烁：TIM2 周期错误
 * 3 次闪烁：TIM3 周期错误
 *
 * 注意：这里使用空循环做延时，不依赖 SysTick。
 */
void User_Error_Handler(uint8_t code)
{

  __disable_irq();
  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
  while (1)
  {
	  /* 按错误代码闪烁对应次数。 */
      for (uint8_t i=0; i<code; i++)
      {

          HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
          for (volatile uint32_t i = 0; i < 800000; i++) {};
          HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
          for (volatile uint32_t i = 0; i < 800000; i++) {};
      }

      /* 每组闪烁之间暂停一段时间。 */
      for (volatile uint32_t i = 0; i < 4000000; i++) {};

  }

}







