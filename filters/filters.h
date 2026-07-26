#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MOVING_AVERAGE_MAX_WINDOW
#define MOVING_AVERAGE_MAX_WINDOW 16U
#endif

#if (MOVING_AVERAGE_MAX_WINDOW == 0U) || \
    (MOVING_AVERAGE_MAX_WINDOW > 65535U)
#error "MOVING_AVERAGE_MAX_WINDOW must be in the range 1..65535"
#endif

typedef struct {
    float alpha;
    float output;
    uint8_t initialized;
} EmaFilter;

typedef struct {
    float buffer[MOVING_AVERAGE_MAX_WINDOW];
    float sum;
    uint16_t window_size;
    uint16_t index;
    uint16_t count;
} MovingAverageFilter;

void EmaFilter_Init(EmaFilter *filter, float alpha, float initial_value);
float EmaFilter_Update(EmaFilter *filter, float input);
void EmaFilter_Reset(EmaFilter *filter, float value);

void MovingAverage_Init(MovingAverageFilter *filter, uint16_t window_size);
float MovingAverage_Update(MovingAverageFilter *filter, float input);
void MovingAverage_Reset(MovingAverageFilter *filter);

float Median3_Filter(float first, float second, float third);

#ifdef __cplusplus
}
#endif

#endif /* FILTERS_H */
