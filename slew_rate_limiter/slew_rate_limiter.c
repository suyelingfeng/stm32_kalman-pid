#include "slew_rate_limiter.h"

#include <stddef.h>

static float PositiveMagnitude(float value)
{
    return (value < 0.0f) ? -value : value;
}

void SlewRateLimiter_Init(SlewRateLimiter *limiter,
                          float rise_rate,
                          float fall_rate,
                          float initial_output)
{
    if (limiter == NULL) {
        return;
    }

    limiter->rise_rate = PositiveMagnitude(rise_rate);
    limiter->fall_rate = PositiveMagnitude(fall_rate);
    limiter->output = initial_output;
}

float SlewRateLimiter_Update(SlewRateLimiter *limiter,
                             float target,
                             float dt_seconds)
{
    float delta;
    float maximum_rise;
    float maximum_fall;

    if (limiter == NULL) {
        return target;
    }
    if (dt_seconds <= 0.0f) {
        return limiter->output;
    }

    delta = target - limiter->output;
    maximum_rise = limiter->rise_rate * dt_seconds;
    maximum_fall = limiter->fall_rate * dt_seconds;

    if (delta > maximum_rise) {
        delta = maximum_rise;
    } else if (delta < -maximum_fall) {
        delta = -maximum_fall;
    }

    limiter->output += delta;
    return limiter->output;
}

void SlewRateLimiter_Reset(SlewRateLimiter *limiter, float output)
{
    if (limiter == NULL) {
        return;
    }
    limiter->output = output;
}
