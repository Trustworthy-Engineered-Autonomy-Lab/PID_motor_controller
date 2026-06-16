#include "hall_sensor.h"
#include "app_config.h"

static volatile float latest_raw_rpm = 0.0f;

static volatile uint32_t hall_capture_value = 0;
static volatile uint32_t debug_hall_capture_prev = 0;
static volatile uint32_t debug_hall_capture_delta = 0;
static volatile uint32_t debug_hall_capture_spike_count = 0;

float hall_sensor_get_raw_rpm(void)
{
    return latest_raw_rpm;
}

uint32_t hall_sensor_get_capture_value(void)
{
    return hall_capture_value;
}

uint32_t hall_sensor_get_capture_delta(void)
{
    return debug_hall_capture_delta;
}

uint32_t hall_sensor_get_capture_spike_count(void)
{
    return debug_hall_capture_spike_count;
}

/*
 * Number of Hall capture edges per mechanical revolution.
 *
 * This value belongs to the Hall sensor module because it is used only by
 * the Hall capture-to-RPM conversion logic.
 */
#define HALL_SENSOR_EDGES_PER_REV    12.0f

/*
 * Converts a TIM4 Hall capture interval count to motor speed in RPM.
 *
 * Calculation:
 *   edge_interval_s = capture_value * (TIM4_PSC + 1) / CLK_FREQ
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
           ((float)capture_value * (TIM4_PSC + 1U) * HALL_SENSOR_EDGES_PER_REV);
}

/*
 * Returns the RPM value corresponding to the maximum TIM4 capture period.
 *
 * This is used as a startup fallback when the first Hall capture occurs
 * while RPM feedback is still zero.
 */
static float hall_sensor_min_motor_rpm(void)
{
    return hall_sensor_capture_to_rpm(TIM4_CTR_PER);
}

/*
 * Indicates whether at least one Hall edge has been captured since the
 * previous TIM4 period-elapsed event.
 *
 * The timeout handler uses this flag to distinguish between normal Hall
 * activity and a no-edge timeout condition.
 */
static volatile uint8_t hall_edge_seen_since_timeout = 0;

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
 * Handles one TIM4 Hall input-capture event.
 *
 * The caller provides the captured TIM4 count value. This function stores
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

    hall_capture_value = capture_value;

    hall_capture_spike_check(hall_capture_value);

    float raw_rpm_now;

    if (latest_raw_rpm > 0.0f)
    {
        raw_rpm_now = hall_sensor_capture_to_rpm(hall_capture_value);
    }
    else
    {
        raw_rpm_now = hall_sensor_min_motor_rpm();
    }

    latest_raw_rpm = raw_rpm_now;
}

/*
 * Handles one TIM4 period-elapsed event for Hall no-edge detection.
 *
 * If no Hall edge has been captured during the latest TIM4 period, the
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
        latest_raw_rpm = 0.0f;
    }

    hall_edge_seen_since_timeout = 0;
}
