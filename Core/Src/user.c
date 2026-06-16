#include "user.h"
#include "app_config.h"
#include "reg.h"
#include "motor_control.h"
#include "lp_filter.h"
#include "hall_sensor.h"
#include <math.h>
#include "main.h"


/* Variable Declarations */

/*
 * Local RPM low-pass filter instance used by the user application loop.
 */
static LP_Filter_t rpm_lp_filter;

volatile float motor_rpm_raw = 0.0f;
volatile float motor_rpm_filtered = 0.0f;
volatile uint32_t debug_filter_update_count = 0;
/* End Variable Definitions */


/* Function Declarations */
void tim_per_check(void);

void user_error_handler(uint8_t count);
/* End Function Declarations */


/* Function Definitions */
/*
 * Initializes the user application layer.
 *
 * This function initializes the register interface, RPM filter,
 * motor-control module, Hall sensor module, and control-loop timer.
 */
void user_init(void)
{
    reg_init();

    lp_filter_init(&rpm_lp_filter, RPM_FILTER_ALPHA);

    motor_control_init();

    hall_sensor_init();

    HAL_TIM_Base_Start_IT(&CONTROL_TIMER_HANDLE);
}

/*
 * Runs one control-loop update.
 *
 * This function is called directly from the control timer callback. It
 * updates the RPM low-pass filter, synchronizes the RPM feedback
 * variables, increments the filter debug counter, and runs one
 * motor-control update.
 */
void user_loop(void)
{

    float raw_rpm = hall_sensor_get_raw_rpm();

    lp_filter_compute(&rpm_lp_filter, raw_rpm);

    motor_rpm_raw = rpm_lp_filter.input;
    motor_rpm_filtered = rpm_lp_filter.output;

    debug_filter_update_count++;

    motor_control_update(motor_rpm_filtered);
}

/*
 * Checks whether the configured timer periods match the expected values.
 *
 * If a timer period is outside the allowed tolerance, the function enters
 * user_error_handler() with a code that identifies the failed timer check.
 *
 * This function is a diagnostic helper. It is not called automatically by
 * user_init() in the current code.
 */
void tim_per_check(void){
    /*
     * TIM1 control-loop period check.
     * Allowed error: 0.1 ms.
     */
	if(fabs(TIM1_PER_MS - EXPECTED_TIM1_PER_MS) > .1){
		user_error_handler(1);
	}

    /*
     * TIM2 PWM period check.
     * Allowed error: 0.1 ms.
     */
	if(fabs(TIM2_PER_MS - EXPECTED_TIM2_PER_MS) > .1){
		user_error_handler(2);
	}

    /*
     * TIM4 Hall timer period check.
     * Allowed error: 1 ms.
     */
	if(fabs(TIM4_PER_MS - EXPECTED_TIM4_PER_MS) > 1){
		user_error_handler(3);
	}
}
/* End Function Definition */


/* Interrupt Functions */
/*
 * Handles timer period-elapsed callbacks.
 *
 * The control timer directly runs one user control-loop update. The Hall
 * timer calls the Hall sensor timeout handler for no-edge detection.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == CONTROL_TIMER_HANDLE.Instance)
    {
        user_loop();
    }

    if (htim->Instance == HALL_TIMER_HANDLE.Instance)
    {
        hall_sensor_timeout_handler();
    }
}

/*
 * Handles timer input-capture callbacks.
 *
 * When a Hall sensor capture event is received, this function reads the
 * configured Hall capture channel and forwards the captured value to the
 * Hall sensor module.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == HALL_TIMER_HANDLE.Instance)
    {
        uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim, HALL_CAPTURE_CHANNEL);

        hall_sensor_capture_handler(capture_value);
    }
}
/* End Interrupt Functions */

/*
 * Handles timer-period diagnostic errors.
 *
 * Interrupts are disabled, and the green LED blinks according to the
 * error code:
 *   1 blink  -> TIM1 period check failed.
 *   2 blinks -> TIM2 period check failed.
 *   3 blinks -> TIM4 period check failed.
 *
 * The delay loops are blocking software delays and do not depend on
 * SysTick.
 */
void user_error_handler(uint8_t code)
{

  __disable_irq();
  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
  while (1)
  {
      /*
       * Blink the LED according to the error code.
       */
      for (uint8_t i=0; i<code; i++)
      {

          HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
          for (volatile uint32_t i = 0; i < 800000; i++) {};
          HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
          for (volatile uint32_t i = 0; i < 800000; i++) {};
      }

      /*
       * Insert a longer pause between blink groups.
       */
      for (volatile uint32_t i = 0; i < 4000000; i++) {};

  }

}







