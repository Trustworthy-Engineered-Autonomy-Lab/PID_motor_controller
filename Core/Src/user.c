
/* Includes */
#include "main.h"
#include <hall_sensor.h>
#include <lp_filter.h>
#include "user.h"
#include "app_config.h"
#include "lp_filter.h"
#include "reg.h"
#include "motor_control.h"
#include <math.h>
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

/* 标志位：由中断置 1，由主循环清 0。 */
volatile uint8_t pwm_update_flag = 0;	// 当前版本暂未使用，保留给后续 PWM 更新逻辑
volatile uint8_t pid_update_flag = 0;	// TIM1 周期触发；当前版本用作 PWM 周期刷新触发
volatile uint8_t speed_update_flag = 0;	// I2C 收到新指令后置 1
volatile uint8_t hall_update_flag = 0;	// 霍尔输入捕获或 TIM3 溢出后置 1

static LP_Filter_t rpm_lp_filter;

/* End Variable Definitions */


/* Function Declarations */
void TIM_PER_CHECK(void);

void User_Error_Handler(uint8_t count);
/* End Function Declarations */


/* Function Definitions */

void User_Init(void)
{
    reg_init();

    lp_filter_init(&rpm_lp_filter, RPM_FILTER_ALPHA);

    Motor_Control_Init();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    Motor_Control_Set_PWM_US(PWM_US_NEUTRAL);

    HAL_TIMEx_HallSensor_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim3);

    HAL_TIM_Base_Start_IT(&htim1);
}

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

        lp_filter_compute(&rpm_lp_filter, latest_raw_rpm);

        motor_rpm_raw = rpm_lp_filter.input;
        motor_rpm_filtered = rpm_lp_filter.output;
        motor_rpm = rpm_lp_filter.output;

        debug_filter_update_count++;

        Motor_Control_Update();
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
/* End Function Definition */


/* Interrupt Functions */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        pid_update_flag = 1;
    }

    if (htim->Instance == TIM3) {
        hall_sensor_timeout_handler();
    }

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        hall_sensor_capture_handler(capture_value);
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







