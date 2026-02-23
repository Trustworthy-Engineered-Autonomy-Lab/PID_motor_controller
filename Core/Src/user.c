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
// FIXME: (Low) Eventual change to fixed point arithmetic (or purchase chip with FPU)
/* FIXME: (High) speed_setpoint should def be float when real speed values are used, kept as int16_t until
 speed to rpm conversion figured out */
volatile int16_t speed_setpoint = 0;
volatile uint8_t pwm_update_flag = 0;
volatile uint8_t pid_update_flag = 0;
volatile uint8_t speed_update_flag = 0;
volatile uint8_t hall_update_flag = 0;
// FIXME: (High) motor_rpm hould probably be int16_t since magnitude on order of hundreds
static volatile float motor_rpm = 0;
volatile uint32_t hall_capture_value = 0;
// value below uses HAL_GetTick() to measure time between last time the motor sent an rpm value
// resolves issue of motor rpm not updating to 0 when motor was not spinning
volatile uint32_t lastHallSensorUpdate = 0;

PID_t motor_pid; // Instantiate motor PID controller

// PID values
// FIXME: (High) Start tuning controller
const float dt = TIM1_PER_MS / 1000; // PID update rate
const float Kp = 0.00001; // Proportional Coefficient
const float Ki = 0.000005; // Integral Coefficient
const float Kd = 0; // Derivative Coefficient
// FIXME: (Medium) Calibrate better integral clamp
const float Integral_max = 100000.0f; // Anti-windup value
const float pid_max = PWM_MAX_PULSEWIDTH - PWM_ZERO_PULSEWIDTH; // 1.5ms +- .5ms


/* End Variable Definitions */

/* Function Declarations */
void TIM_PER_CHECK(void);

// FIXME: (Medium) Update rpm/speed_setpoint types later
float rpm_update(int16_t speed_setpoint);
void openloop_pwm_update(float rpm_setpoint);
void pid_pwm_update(float rpm_setpoint);
uint32_t rpm_to_pwm_duty(int16_t rpm_setpoint);
void User_Error_Handler(uint8_t count);


/* End Function Declarations */

/* Function Definitions */

void User_Init(void){ // Initialization function
	TIM_PER_CHECK();

	HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&speed_setpoint, 2); // FIXME: Why is this here?

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_CCR_DEFAULT); // Default pulse width = 1.5ms

	HAL_TIMEx_HallSensor_Start_IT(&htim3); // Start timer 3 with IC Interrupt
	HAL_TIM_Base_Start_IT(&htim3); // Starts timer 3 overflow interrupt

	HAL_TIM_Base_Start_IT(&htim1); // start timer 1


	// Initialize PID controller with coefficient values
	PID_Init(&motor_pid,
			Kp,
			Ki,
			Kd,
			Integral_max,
			pid_max
			);
}

void User_Loop(void){ // Main Loop
	// FIXME: (Medium) rpm_setpoint should probably be uint16_t
	static float rpm_setpoint; // Static to stay the same until new i2c command

	// Check if new speed target has been received
	if (speed_update_flag == 1){
		speed_update_flag = 0;
		rpm_setpoint = rpm_update(speed_setpoint);
	}

	// Checks if there is a hall sensor update
	if (hall_update_flag == 1){
		// FIXME: Add computation here as well
		hall_update_flag = 0;
	}

	// Check if new I2C data received and PWM needs updating
	if (pid_update_flag == 1){ // pid updated every 10ms via TIM1
		pid_update_flag = 0;
		pid_pwm_update(rpm_setpoint);
	}
	// Note: Starts ESC at 1.5ms pulse width, then stay between 1ms and 2ms
}

// Checks that timer periods generated from CubeMX are what we want them to be
void TIM_PER_CHECK(void){
	// Checks if error within .1ms
	if(fabs(TIM1_PER_MS - EXPECTED_TIM1_PER_MS) > .1){
		User_Error_Handler(1);
	}

	// Checks if error within .1ms
	if(fabs(TIM2_PER_MS - EXPECTED_TIM2_PER_MS) > .1){
		User_Error_Handler(2);
	}

	// Checks if error within 1ms
	if(fabs(TIM3_PER_MS - EXPECTED_TIM3_PER_MS) > 1){
		User_Error_Handler(3);
	}


}


// Converts speed command to RPM for PID controller
float rpm_update(int16_t speed_setpoint){
	// FIXME: (Low) Calibrate conversion once car can be set up
	// For now, directly equate values (i2c will need to receive rpm instead of speed)
	return (float)speed_setpoint;
}


/**
 * @brief Update PWM duty cycle based on current RPM setpoint
 * @retval None
 *
 * Call this function in main loop when pwm_update_flag is set.
 */

/* FIXME: (Low) Update function to receive pulse width or PWM or something.
 * Would require larger structural changes (i.e. not using i2c) to basically pass
 * pulse width directly from Jetson Nano to ESC. Kept as is for now for debugging
 * and prototyping purposes.
 */
void openloop_pwm_update(float rpm_setpoint) // non_pid algo
{
	/* FIXME: (High) Even for debugging purposes, basic open loop rpm to pulse width conversion
	needed so we can start sending speed values over Jetson instead of changing what we send
	based off of if we use the open loop or closed loop functions. */

	/*
    Code below not used since we're taking in raw CCR values for now
    uint32_t new_pwm_value;

    Convert RPM to PWM duty cycle
	new_pwm_value = rpm_to_pwm_duty(rpm_setpoint);

	Update PWM compare value (this changes the duty cycle)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, new_pwm_value);

    used for later development, for now we will directly read CCR from I2C
    Convert RPM to PWM duty cycle
	new_pwm_value = rpm_to_pwm_duty(rpm_setpoint);

     */



	uint32_t new_ccr_value;
    // Ensure CCR is within valid range
	if (rpm_setpoint < PWM_CCR_MIN) {
		new_ccr_value = PWM_CCR_MIN;
	} else if (rpm_setpoint > PWM_CCR_MAX) {
		new_ccr_value = PWM_CCR_MAX;
	} else {
		new_ccr_value = (uint32_t)rpm_setpoint;
	}

    // Update PWM compare value (this changes the duty cycle)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, new_ccr_value);

    // Optional: Add debugging output (remove in production)
    // printf("RPM: %d, PWM: %lu\r\n", rpm_setpoint, new_pwm_value);
}


void pid_pwm_update(float rpm_setpoint) {

	/*
	 * motor_rpm set in TIM3 Hall Sensor Trigger interrupt
	 * rpm_setpoint set within I2C receive callback function
	 * Calculate error from these two values
	 * Use error to find P, I, & D
	 * Calculate set point via the tuned PID weights
	 * Use this set point in update_pwm to output signal to motor driver
	 */

	// Compute PID output. PID output is a pulse width adjustment
	// FIXME: (Low) Update PID to compute with fixed point instead of float?
	PID_Compute(&motor_pid, rpm_setpoint, motor_rpm, dt);


	// Computes new pulse_width value and then converts to CCR value
	float pulse_width = PWM_ZERO_PULSEWIDTH + motor_pid.output;
	uint32_t ccr_output = (uint32_t)PWM_PULSEWIDTH_TO_CCR(pulse_width);

	/* FIXME: (Low) Clamp CCR output. Minimum set to 1.5ms due to lack of reversing logic
	Might not be necessary once reversing logic added since PID output is already clamped */
	// FIXME: (Low) Add Reversing Logic
	if(ccr_output < PWM_CCR_DEFAULT) {
		ccr_output = PWM_CCR_DEFAULT;
	} else if (ccr_output > PWM_CCR_MAX) {
		ccr_output = PWM_CCR_MAX;
	}

	// Update PWM
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_output);

}

/**
 * @brief Convert RPM setpoint to PWM duty cycle
 * @param rpm: RPM value received from I2C (0 to RPM_MAX_VALUE)
 * @retval PWM compare value (0 to PWM_MAX_VALUE)
 *
 * This function maps the RPM value to appropriate PWM duty cycle for open loop control.
 * You can modify this function to implement different control algorithms.
 * ***Unneeded for now since we are taking in raw CCR values
 */
/* FIXME: (High) Update function (as explained in openloop_pwm_update)
 Test some ccr values and rpm outputs to get a linear model with regression */
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        pid_update_flag = 1;  // Signal main loop to run PID
    }

    if (htim->Instance == TIM3) {
    	if (hall_update_flag == 0){
    		// Checks that update wasn't triggered by IC
    		motor_rpm = 0;  // hall sensor timeout -> rpm set to 0
    		hall_update_flag = 1;
    	}

    }

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){

	// Re-arm the I2C to receive the next byte
	HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&speed_setpoint, 2);

	/* I2C Transmitting only used for testing purposes */
	//	HAL_I2C_Slave_Transmit_IT(&hi2c1, &rpm_setpoint, 2);

	speed_update_flag = 1;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	hall_update_flag = 1;

	// FIXME: (Low) Update interrupt priority to be greater than PID_update

	if (htim->Instance == TIM3) { // Must make sure we're only processing TIM3's events
        // Hall sensor capture event
		//hall_capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Reads from capture channel 1, same as line below
        hall_capture_value = htim->Instance->CCR1; // Compare/Capture Register 1 store # of ticks between transitions

        // FIXME: (Medium) divide by 2.0f bc RPM too high?? TBD (MAX rpm = ~38k, should be ~20k -> Find way to check hall sensor accuracy
        // FIXME: (High) Computation needs to be moved outside interrupt
        if (hall_capture_value > 0) {
        	// 1. Convert time between hall sensor edge transitions from # of ticks
            float time_between_edges = (float)hall_capture_value * TIM2_PSC / CLK_FREQ;
            // 2. Store frequency of hall sensor edge transitions as # of ticks per transition
            float frequency = 1.0f / time_between_edges;
            // 3. Convert to Rotations per Minute
            // 6 transitions per 1 full rotation | 60 seconds in a minute
            // # of ticks per transition * 60 seconds / minute / (6 transitions/rotation) = Rotation per Minute
            // FIXME: (Low) Update output to be integer
            motor_rpm = (frequency * 60.0f) / 6.0f;
        }
    }
}

/* End Interrupt Functions */

// Custom Error Handler
// FIXME: (Low) More robust error handler (set it up to send a message with UART or SWO instead of just blinking an LED)
/*
 * Codes:
 * 1 blink: TIM1 Period wrong
 * 2 blink: TIM2 Period wrong
 * 3 blink: TIM3 Period wrong
 */
void User_Error_Handler(uint8_t code)
{

  __disable_irq(); // Block loops needed because no SysTick interrupts
  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET); // Turn LED Off
  while (1) // Blinks Green LED to display error
  {
      // blink LED 'code' times
      for (uint8_t i=0; i<code; i++)
      {

          HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
          for (volatile uint32_t i = 0; i < 800000; i++) {}; // Very rough estimate of about 200ms
          HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
          for (volatile uint32_t i = 0; i < 800000; i++) {}; // Very rough estimate of about 200ms
      }
      for (volatile uint32_t i = 0; i < 4000000; i++) {}; // Very rough estimate of about 1s

  }

}







