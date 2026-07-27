#include "filters.h"

#include <stddef.h>

static float ClampUnit(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

void EmaFilter_Init(EmaFilter *filter, float alpha, float initial_value)
{
    if (filter == NULL) {
        return;
    }

    filter->alpha = ClampUnit(alpha);
    filter->output = initial_value;
    filter->initialized = 1U;
}

float EmaFilter_Update(EmaFilter *filter, float input)
{
    if (filter == NULL) {
        return input;
    }

    if (filter->initialized == 0U) {
        filter->output = input;
        filter->initialized = 1U;
        return input;
    }

    /* EMA: alpha 越小越平滑，越大越快跟随新样本。 */
    filter->output += filter->alpha * (input - filter->output);
    return filter->output;
}

void EmaFilter_Reset(EmaFilter *filter, float value)
{
    if (filter == NULL) {
        return;
    }

    filter->output = value;
    filter->initialized = 1U;
}

void MovingAverage_Init(MovingAverageFilter *filter, uint16_t window_size)
{
    if (filter == NULL) {
        return;
    }

    if (window_size == 0U) {
        window_size = 1U;
    } else if (window_size > (uint16_t)MOVING_AVERAGE_MAX_WINDOW) {
        window_size = (uint16_t)MOVING_AVERAGE_MAX_WINDOW;
    }

    filter->window_size = window_size;
    MovingAverage_Reset(filter);
}

float MovingAverage_Update(MovingAverageFilter *filter, float input)
{
    if ((filter == NULL) || (filter->window_size == 0U)) {
        return input;
    }

    /* 窗口填满后，用新样本替换最旧样本并修正累计和。 */
    if (filter->count == filter->window_size) {
        filter->sum -= filter->buffer[filter->index];
    } else {
        ++filter->count;
    }

    filter->buffer[filter->index] = input;
    filter->sum += input;
    ++filter->index;
    if (filter->index == filter->window_size) {
        filter->index = 0U;
    }

    return filter->sum / (float)filter->count;
}

void MovingAverage_Reset(MovingAverageFilter *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->sum = 0.0f;
    filter->index = 0U;
    filter->count = 0U;
}

/* 固定三次比较即可得到中值，避免通用排序的额外开销。 */
float Median3_Filter(float first, float second, float third)
{
    float temporary;

    if (first > second) {
        temporary = first;
        first = second;
        second = temporary;
    }
    if (second > third) {
        temporary = second;
        second = third;
        third = temporary;
    }
    if (first > second) {
        second = first;
    }

    return second;
}
