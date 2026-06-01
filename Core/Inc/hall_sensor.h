#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void hall_sensor_init(void);

/*
 * Updates Hall capture spike debug variables.
 *
 * This function is used for debugging only. It does not directly change
 * motor speed feedback or control output.
 */
void hall_capture_spike_check(uint32_t capture_value);

/*
 * Handles a TIM3 Hall input-capture event.
 *
 * The caller passes in the captured TIM3 count value. In the current
 * project, this function is called from HAL_TIM_IC_CaptureCallback()
 * after reading the configured Hall capture channel.
 */
void hall_sensor_capture_handler(uint32_t capture_value);

/*
 * Handles a TIM3 Hall timer period-elapsed event.
 *
 * In the current project, this function is called from the TIM3 branch
 * of HAL_TIM_PeriodElapsedCallback() and applies the Hall no-edge
 * timeout logic.
 */
void hall_sensor_timeout_handler(void);

float hall_sensor_get_raw_rpm(void);
uint32_t hall_sensor_get_capture_value(void);
uint32_t hall_sensor_get_capture_delta(void);
uint32_t hall_sensor_get_capture_spike_count(void);

#ifdef __cplusplus
}
#endif

#endif /* HALL_SENSOR_H */
