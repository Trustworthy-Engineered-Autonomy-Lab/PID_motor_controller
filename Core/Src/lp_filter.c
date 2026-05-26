#include "lp_filter.h"

/*
 * Initializes a first-order low-pass filter instance.
 *
 * The function clears the input, output, and initialization state.
 * A negative alpha is converted to its absolute value. An alpha greater
 * than 1.0f is clamped to 1.0f.
 */
void lp_filter_init(LP_Filter_t *filter,
                    float alpha)
{
    if (filter == 0)
    {
        return;
    }

    if (alpha < 0.0f)
    {
        alpha = -alpha;
    }

    if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    filter->alpha = alpha;
    filter->input = 0.0f;
    filter->output = 0.0f;
    filter->initialized = 0;
}

/*
 * Computes one first-order low-pass filter update.
 *
 * The latest input sample is always stored in filter->input.
 *
 * If the input is less than or equal to zero, the filter output is
 * cleared and the filter is marked uninitialized.
 *
 * If the filter is not initialized, the first positive input sample is
 * copied directly to the output. After initialization, the output is
 * updated with:
 *
 *   output = output + alpha * (input - output)
 */
void lp_filter_compute(LP_Filter_t *filter,
                       float input)
{
    if (filter == 0)
    {
        return;
    }

    filter->input = input;

    if (input <= 0.0f)
    {
        filter->output = 0.0f;
        filter->initialized = 0;
        return;
    }

    if (filter->initialized == 0)
    {
        filter->output = input;
        filter->initialized = 1;
    }
    else
    {
        filter->output = filter->output +
                         filter->alpha * (input - filter->output);
    }
}
