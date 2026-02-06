/* Includes */
#include "user.h"


/* End Includes */


/* Variable Declarations */

// Pull data types from main.c
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;


// make following variables volatile since they are altered within constantly by ISR
volatile int16_t rpm_set;
volatile uint8_t pwm_update_flag = 0;
volatile uint8_t pid_update_flag = 0;
volatile float motor_rpm = 0;
volatile uint32_t hall_capture_value = 0;

PID_t motor_pid; // Instantiate motor PID controller

// PID values
const float dt = 0.015f; // PID update rate
const float Kp = 0.1; // Proportional Coefficient
const float Ki = 0; // Integral Coefficient
const float Kd = 0; // Derivative Coefficient
const float Integral_max = 500.0f; // Anti-windup value

/* End Variable Definitions */

/* Function Declarations */

//uint32_t rpm_to_pwm_duty(int16_t rpm);
void openloop_pwm_update(void);
void pid_pwm_update(void);

/* End Function Declarations */

/* Function Definitions */

void User_Init(void){ // Initialization function

	HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&rpm_set, 2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	//  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 18000-1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PULSE_WIDTH_TO_CCR(1.5));
	//  HAL_I2C_Slave_Transmit_IT(&hi2c1, /*(uint8_t*)*/&rpm_set, 2);
	//  HAL_Delay(100);
	HAL_TIMEx_HallSensor_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim1); // start timer 1


	// Initialize PID controller with coefficient values
	PID_Init(&motor_pid,
			Kp,
			Ki,
			Kd,
			Integral_max
			);
}

void loop(void){ // Main Loop

	//	  openloop_pwm_update();
	// Check if new I2C data received and PWM needs updating
	if (pid_update_flag == 1){ // pid updated every 10ms via TIM1
		pid_update_flag = 0;
		pid_pwm_update();
	}
}



/**
 * @brief Update PWM duty cycle based on current RPM setpoint
 * @retval None
 *
 * Call this function in main loop when pwm_update_flag is set.
 */
void openloop_pwm_update(void) // non_pid algo
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

    // Optional: Add debugging output (remove in production)
    // printf("RPM: %d, PWM: %lu\r\n", rpm_set, new_pwm_value);
}


void pid_pwm_update(void) {

	/*
	 * motor_rpm set in TIM3 Hall Sensor Trigger interrupt
	 * rpm_set set within I2C receive callback function
	 * Calculate error from these two values
	 * Use error to find P, I, & D
	 * Calculate set point via the tuned PID weights
	 * Use this set point in update_pwm to output signal to motor driver
	 */

	float pid_output = PID_Compute(&motor_pid, (float)rpm_set, motor_rpm, dt);


	// Converting PID output to Compare & Capture value for PWM
	// FIXME: relationship between pid_output unknown
	// FOR NOW: will just add float value to base CCR

	float base_ccr = PULSE_WIDTH_TO_CCR(1.5);
	float new_ccr = base_ccr + pid_output;

	// 7. Clamp CCR
	uint32_t ccr_output;
	if(new_ccr < 0) {
		ccr_output = 0;
	} else if (new_ccr > CCR_MAX_VALUE) {
		ccr_output = CCR_MAX_VALUE;
	} else {
		ccr_output = (uint32_t)new_ccr;
	}

	// Update PWM
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_output);

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

/* End Function Definition */

/* Interrupt Functions */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        pid_update_flag = 1;  // Signal main loop to run PID
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
//	HAL_GPIO_WritePin(GPIOC, LED_Green_Pin, GPIO_PIN_RESET);
////	HAL_Delay(200);
//	HAL_GPIO_WritePin(GPIOC, LED_Green_Pin, GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOC, LED_Green_Pin);

//	pwm_update_flag = 1; // not needed anymore, setpoint update in callback function (i.e. interrupt)

	// Re-arm the I2C to receive the next byte
	HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&rpm_set, 2);
//	HAL_I2C_Slave_Transmit_IT(&hi2c1, &rpm_set, 2); /* I2C Transmitting only used for testing purposes */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* FIXME: May be an issue that this only updates when TIM3 is triggered by the
     * motor spinning?? i.e. when the wheel stops does the controller incorrectly
     * output an rpm that's *not* 0?
     */
	/*
	 * FIXME: Potential fix: use HAL_GetTick() to see how long it has been since last trigger.
	 * If it's been more than X ms then set motor_rpm = 0
	 */
	if (htim->Instance == TIM3) { // Must make sure we're only processing TIM3's events
        // Hall sensor capture event
//        hall_capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Reads from capture channel 1, same as line below
        hall_capture_value = htim->Instance->CCR1; // Compare/Capture Register 1 store # of ticks between transitions

        // FIXME: divide by 2.0f bc RPM too high?? TBD (MAX rpm = ~38k, should be ~20k
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

/* End Interrupt Functions */







