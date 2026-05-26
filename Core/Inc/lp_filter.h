#ifndef LP_FILTER_H
#define LP_FILTER_H

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * First-order low-pass filter state.
 *
 * alpha controls the filter response:
 *   alpha = 0.0f keeps the previous output unchanged.
 *   alpha = 1.0f makes the output follow the input immediately.
 *
 * input stores the latest input sample.
 * output stores the latest filtered output.
 * initialized indicates whether the first valid sample has been loaded.
 */
typedef struct
{
    float alpha;
    float input;
    float output;
    unsigned char initialized;

} LP_Filter_t;

/*
 * Raw RPM value used by the motor-control feedback path.
 *
 * In the current project, this value is assigned from the filter input
 * in user_loop(). The Hall sensor module may also clear or update it
 * when capture or timeout events occur.
 */
extern volatile float motor_rpm_raw;

/*
 * Filtered RPM value used for monitoring and feedback.
 *
 * In the current project, this value is assigned from the filter output
 * in user_loop(). The Hall sensor timeout handler may clear it when no
 * Hall edge is detected during a timeout period.
 */
extern volatile float motor_rpm_filtered;

/*
 * Backward-compatible RPM feedback variable.
 *
 * In the current project, user_loop() keeps this value synchronized with
 * motor_rpm_filtered, and the motor-control module uses it as the RPM
 * feedback value.
 */
extern volatile float motor_rpm;

/*
 * Latest raw RPM value produced by the Hall sensor capture path.
 *
 * The Hall sensor capture handler writes this value. The control loop in
 * user_loop() passes it into lp_filter_compute() when the control update
 * flag is set.
 */
extern volatile float latest_raw_rpm;

/*
 * Debug counter for RPM filter updates.
 *
 * This counter is incremented in user_loop() each time the RPM filter is
 * computed during a control update.
 */
extern volatile uint32_t debug_filter_update_count;

/*
 * Initializes a first-order low-pass filter instance.
 *
 * If the filter pointer is null, the function returns without changing
 * any state. A negative alpha is converted to its absolute value, and an
 * alpha greater than 1.0f is clamped to 1.0f.
 */
void lp_filter_init(LP_Filter_t *filter,
                    float alpha);

/*
 * Computes one first-order low-pass filter update.
 *
 * If the filter pointer is null, the function returns without changing
 * any state. If the input is less than or equal to zero, the output is
 * cleared and the filter is marked uninitialized. The next positive
 * input sample initializes the output directly.
 */
void lp_filter_compute(LP_Filter_t *filter,
                       float input);

#ifdef __cplusplus
}
#endif

#endif /* LP_FILTER_H */
