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
