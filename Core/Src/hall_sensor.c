#include "hall_sensor.h"
#include "app_config.h"

/*
 * These RPM feedback variables are defined in motor_control.c.
 * hall_sensor.c only uses them to update or clear RPM feedback.
 *
 * Local extern declarations avoid adding an unnecessary dependency on
 * lp_filter.h or introducing another shared header.
 */
extern volatile float motor_rpm_raw;
extern volatile float motor_rpm_filtered;
extern volatile float motor_rpm;
extern volatile float latest_raw_rpm;

/*
 * Number of Hall capture edges per mechanical revolution.
 *
 * This value belongs to the Hall sensor module because it is used only by
 * the Hall capture-to-RPM conversion logic.
 */
#define HALL_SENSOR_EDGES_PER_REV    12.0f

/*
 * Converts a TIM3 Hall capture interval count to motor speed in RPM.
 *
 * Calculation:
 *   edge_interval_s = capture_value * (TIM3_PSC + 1) / CLK_FREQ
 *   edge_frequency  = 1 / edge_interval_s
 *   motor_rpm       = edge_frequency * 60 / HALL_SENSOR_EDGES_PER_REV
 */
static float hall_sensor_capture_to_rpm(uint32_t capture_value)
{
    if (capture_value == 0U)
    {
        return 0.0f;
    }

    return (CLK_FREQ * 60.0f) /
           ((float)capture_value * (TIM3_PSC + 1U) * HALL_SENSOR_EDGES_PER_REV);
}

/*
 * Returns the RPM value corresponding to the maximum TIM3 capture period.
 *
 * This is used as a startup fallback when the first Hall capture occurs
 * while RPM feedback is still zero.
 */
static float hall_sensor_min_motor_rpm(void)
{
    return hall_sensor_capture_to_rpm(TIM3_CTR_PER);
}

/*
 * Indicates whether at least one Hall edge has been captured since the
 * previous TIM3 period-elapsed event.
 *
 * The timeout handler uses this flag to distinguish between normal Hall
 * activity and a no-edge timeout condition.
 */
static volatile uint8_t hall_edge_seen_since_timeout = 0;

/*
 * Latest TIM3 Hall capture count used for RPM conversion.
 *
 * In the current configuration, the capture value is treated as the
 * timer-count interval associated with the latest Hall edge.
 */
volatile uint32_t hall_capture_value = 0;

/*
 * Hall sensor update timestamp placeholder.
 *
 * This variable is currently defined for external access, but this
 * module does not update it.
 */
volatile uint32_t last_hall_sensor_update = 0;

/*
 * Debug variables for detecting large Hall capture-value changes.
 *
 * debug_hall_capture_prev stores the previous capture value.
 * debug_hall_capture_delta stores the absolute difference between the
 * current and previous capture values.
 * debug_hall_capture_spike_count counts capture values that are much
 * smaller or larger than the previous value.
 */
volatile uint32_t debug_hall_capture_prev = 0;
volatile uint32_t debug_hall_capture_delta = 0;
volatile uint32_t debug_hall_capture_spike_count = 0;

/*
 * Initializes the Hall sensor module.
 *
 * This starts the Hall input-capture interrupt and the Hall timer base
 * interrupt. The input-capture interrupt is used to measure motor speed,
 * while the base interrupt is used for no-edge timeout detection.
 */
void hall_sensor_init(void)
{
    HAL_TIMEx_HallSensor_Start_IT(&HALL_TIMER_HANDLE);
    HAL_TIM_Base_Start_IT(&HALL_TIMER_HANDLE);
}

/*
 * Updates Hall capture spike debug data.
 *
 * This function is used only for debugging and does not directly affect
 * RPM feedback, filtering, PID control, or PWM output.
 *
 * A spike is counted when the current capture value is less than half of
 * the previous value or greater than twice the previous value.
 */
void hall_capture_spike_check(uint32_t capture_value)
{
    if (debug_hall_capture_prev > 0)
    {
        if (capture_value > debug_hall_capture_prev)
        {
            debug_hall_capture_delta = capture_value - debug_hall_capture_prev;
        }
        else
        {
            debug_hall_capture_delta = debug_hall_capture_prev - capture_value;
        }

        /*
         * Count a large relative jump between two consecutive capture
         * values for debugging.
         */
        if ((capture_value < (debug_hall_capture_prev / 2)) ||
            (capture_value > (debug_hall_capture_prev * 2)))
        {
            debug_hall_capture_spike_count++;
        }
    }

    debug_hall_capture_prev = capture_value;
}

/*
 * Handles one TIM3 Hall input-capture event.
 *
 * The caller provides the captured TIM3 count value. This function stores
 * the latest capture value, updates capture-jump debug data, converts the
 * capture value to a raw RPM estimate when valid previous RPM feedback is
 * available, and stores the latest raw RPM for the control-loop filter.
 *
 * Filtering is intentionally not performed in this capture handler. The
 * RPM low-pass filter is updated later in the control loop.
 */
void hall_sensor_capture_handler(uint32_t capture_value)
{
	hall_edge_seen_since_timeout = 1;

    /*
     * Store the latest capture value for RPM conversion and debugging.
     */
    hall_capture_value = capture_value;

    /*
     * Update debug-only capture-jump statistics.
     */
    hall_capture_spike_check(hall_capture_value);

    float raw_rpm_now;

    if (motor_rpm_raw > 0.0f || motor_rpm_filtered > 0.0f)
    {
        raw_rpm_now = hall_sensor_capture_to_rpm(hall_capture_value);
    }
    else
    {
        raw_rpm_now = hall_sensor_min_motor_rpm();
    }

    /*
     * Store the latest raw RPM. The filter update is handled by the
     * control loop rather than by this interrupt-side capture handler.
     */
    latest_raw_rpm = raw_rpm_now;
    motor_rpm_raw = raw_rpm_now;
}

/*
 * Handles one TIM3 period-elapsed event for Hall no-edge detection.
 *
 * If no Hall edge has been captured during the latest TIM3 period, the
 * motor RPM feedback values are cleared. Otherwise, the captured edge is
 * treated as valid activity and the RPM values are left unchanged.
 *
 * The edge-seen flag is reset at the end of each period so the next
 * timeout window can be evaluated independently.
 */
void hall_sensor_timeout_handler(void)
{
    if (hall_edge_seen_since_timeout == 0)
    {
        /*
         * No Hall edge was captured during this timeout window. Clear all
         * RPM feedback values so the control loop sees the motor as stopped.
         */
        latest_raw_rpm = 0.0f;
        motor_rpm_raw = 0.0f;
        motor_rpm_filtered = 0.0f;
        motor_rpm = 0.0f;
    }

    /*
     * Start a new Hall edge detection window after each TIM3 period.
     */
    hall_edge_seen_since_timeout = 0;
}
