#include <lp_filter.h>

/*
 * RPM 一阶低通滤波函数。
 *
 * 输入：
 *   raw_rpm：由单次霍尔捕获计算得到的原始瞬时 RPM。
 *
 * 输出：
 *   motor_rpm_filtered：滤波后的 RPM。
 *   motor_rpm：同步为滤波后的 RPM，供旧逻辑和 PID 使用。
 *
 * 滤波公式：
 *   filtered = filtered + alpha * (raw - filtered)
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
