/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX_DUTY        50      // Maximum PWM compare value (adjust based on your timer config)
#define MAX_PULSEWIDTH		2.0f	// 2ms = Max pulsewidth servo will handle
#define TOTAL_PERIOD		50.0f	// 1 / 20 Hz = 50ms
#define MAX_CCR				36000	// Max CCR = Auto Reload Register Value (the Maximum value our timer will count to)
#define CCR_MAX_VALUE       (MAX_PULSEWIDTH/TOTAL_PERIOD)*MAX_CCR - 1 // 1440 is the corresponding 4% CCR value
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PULSE_WIDTH_TO_DUTYCYCLE(p) ((p)/50.0f)
#define DUTYCYCLE_TO_CCR(d) ((d)*36000-1)
#define PULSE_WIDTH_TO_CCR(p) DUTYCYCLE_TO_CCR(PULSE_WIDTH_TO_DUTYCYCLE(p))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// make following variables volatile since they are altered within constantly by ISR
volatile int16_t rpm_set;
volatile uint8_t pwm_update_flag = 0;
volatile float motor_rpm = 0;
volatile uint32_t hall_capture_value = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//uint32_t rpm_to_pwm_duty(int16_t rpm);
void update_pwm_duty_cycle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&rpm_set, 2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 18000-1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PULSE_WIDTH_TO_CCR(1.5));
//  HAL_I2C_Slave_Transmit_IT(&hi2c1, /*(uint8_t*)*/&rpm_set, 2);
//  HAL_Delay(100);
  HAL_TIMEx_HallSensor_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Check if new I2C data received and PWM needs updating
	  if (pwm_update_flag == 1)
		  update_pwm_duty_cycle(); // Shoutout Claude
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 16;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 35999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Green_Pin */
  GPIO_InitStruct.Pin = LED_Green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_Green_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
//	HAL_GPIO_WritePin(GPIOC, LED_Green_Pin, GPIO_PIN_RESET);
////	HAL_Delay(200);
//	HAL_GPIO_WritePin(GPIOC, LED_Green_Pin, GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOC, LED_Green_Pin);

	pwm_update_flag = 1;

	// Re-arm the I2C to receive the next byte
	HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&rpm_set, 2);
//	HAL_I2C_Slave_Transmit_IT(&hi2c1, &rpm_set, 2); /* I2C Transmitting only used for testing purposes */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) { // Must make sure we're only processing TIM3's events
        // Hall sensor capture event
//        hall_capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Reads from capture channel 1, same as line below
        hall_capture_value = htim->Instance->CCR1; // Compare/Capture Register 1 store # of ticks between transitions

        // TODO: divide by 2.0f bc RPM too high?? TBD (MAX rpm = ~38k, should be ~20k
        if (hall_capture_value > 0) {
        	// 1. Convert time between hall sensor edge transitions from # of ticks
        	// Freq = 720kHz | 1/Freq = time per clock tick = 1.39us | # of ticks between edges * seconds/tick = # of seconds between edges
            float time_between_edges = hall_capture_value / 720000.0f;
            // 2. Store frequency of hall sensor edge transitions as # of ticks per transition
            float frequency = 1.0f / time_between_edges;
            // 3. Convert to Rotations per Minute
            // 6 transitions per 1 full rotation | 60 seconds in a minute
            // # of ticks per transition * 60 seconds / minute / (6 transitions/rotation) = Rotation per Minute
            motor_rpm = (frequency * 60.0f) / 6.0f;
        }
    }
}

/**
 * @brief Convert RPM setpoint to PWM duty cycle
 * @param rpm: RPM value received from I2C (0 to RPM_MAX_VALUE)
 * @retval PWM compare value (0 to PWM_MAX_VALUE)
 *
 * This function maps the RPM value to appropriate PWM duty cycle.
 * You can modify this function to implement different control algorithms.
 * ***Unneeded for now since we are taking in raw CCR values
 */
//uint32_t rpm_to_pwm_duty(int16_t rpm)
//{
//    uint32_t pwm_value;
//
//    // Ensure RPM is within valid range
//    if (rpm < 0) {
//        rpm = 0;
//    } else if (rpm > RPM_MAX_VALUE) {
//        rpm = RPM_MAX_VALUE;
//    }
//
//    // Linear mapping: PWM = (RPM / RPM_MAX) * PWM_MAX
//    pwm_value = ((uint32_t)rpm * PWM_MAX_DUTY) / RPM_MAX_VALUE;
//
//    // Ensure PWM value is within bounds
//    if (pwm_value > PWM_MAX_DUTY) {
//        pwm_value = PWM_MAX_DUTY;
//    }
//
//    return pwm_value;
//}



/**
 * @brief Update PWM duty cycle based on current RPM setpoint
 * @retval None
 *
 * Call this function in your main loop when pwm_update_flag is set.
 */
void update_pwm_duty_cycle(void)
{
    /* Code below not used since we're taking in raw CCR values for now
    uint32_t new_pwm_value;

    // Convert RPM to PWM duty cycle
	new_pwm_value = rpm_to_pwm_duty(rpm_set);

	// Update PWM compare value (this changes the duty cycle)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, new_pwm_value);

    // used for later development, for now we will directly read CCR from I2C
    // Convert RPM to PWM duty cycle
//    new_pwm_value = rpm_to_pwm_duty(rpm_set);

     */

	uint32_t new_ccr_value;
    // Ensure CCR is within valid range
	if (rpm_set < 0) {
		new_ccr_value = 0;
	} else if (rpm_set > CCR_MAX_VALUE) {
		new_ccr_value = CCR_MAX_VALUE;
	} else {
		new_ccr_value = rpm_set;
	}

    // Update PWM compare value (this changes the duty cycle)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, new_ccr_value);

    // Clear the update flag
    pwm_update_flag = 0;

    // Optional: Add debugging output (remove in production)
    // printf("RPM: %d, PWM: %lu\r\n", rpm_set, new_pwm_value);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
