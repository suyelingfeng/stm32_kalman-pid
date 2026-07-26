#include "complementary_filter.h"

#include <stddef.h>

static float ClampAlpha(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

void ComplementaryFilter_Init(ComplementaryFilter *filter,
                              float alpha,
                              float initial_angle)
{
    if (filter == NULL) {
        return;
    }

    filter->alpha = ClampAlpha(alpha);
    filter->angle = initial_angle;
}

float ComplementaryFilter_Update(ComplementaryFilter *filter,
                                 float gyro_rate,
                                 float reference_angle,
                                 float dt_seconds)
{
    float predicted_angle;

    if (filter == NULL) {
        return reference_angle;
    }
    if (dt_seconds < 0.0f) {
        dt_seconds = 0.0f;
    }

    predicted_angle = filter->angle + gyro_rate * dt_seconds;
    filter->angle = filter->alpha * predicted_angle +
                    (1.0f - filter->alpha) * reference_angle;
    return filter->angle;
}

void ComplementaryFilter_Reset(ComplementaryFilter *filter, float angle)
{
    if (filter == NULL) {
        return;
    }
    filter->angle = angle;
}
