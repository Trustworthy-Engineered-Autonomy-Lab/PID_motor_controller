#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Latest TIM3 Hall capture count used for RPM conversion.
 *
 * This value is updated by hall_sensor_capture_handler(), which is
 * called from the input-capture callback path.
 */
extern volatile uint32_t hall_capture_value;

/*
 * Hall sensor update timestamp variable.
 *
 * This variable is currently declared and defined for external access,
 * but the current Hall sensor implementation does not update it.
 */
extern volatile uint32_t last_hall_sensor_update;

/*
 * Debug variables used to monitor large changes in Hall capture values.
 *
 * debug_hall_capture_prev stores the previous capture value.
 * debug_hall_capture_delta stores the absolute difference between the
 * current and previous capture values.
 * debug_hall_capture_spike_count counts large capture-value jumps.
 */
extern volatile uint32_t debug_hall_capture_prev;
extern volatile uint32_t debug_hall_capture_delta;
extern volatile uint32_t debug_hall_capture_spike_count;

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

#ifdef __cplusplus
}
#endif

#endif /* HALL_SENSOR_H */
