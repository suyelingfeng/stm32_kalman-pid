#ifndef SLEW_RATE_LIMITER_H
#define SLEW_RATE_LIMITER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float rise_rate;
    float fall_rate;
    float output;
} SlewRateLimiter;

void SlewRateLimiter_Init(SlewRateLimiter *limiter,
                          float rise_rate,
                          float fall_rate,
                          float initial_output);
float SlewRateLimiter_Update(SlewRateLimiter *limiter,
                             float target,
                             float dt_seconds);
void SlewRateLimiter_Reset(SlewRateLimiter *limiter, float output);

#ifdef __cplusplus
}
#endif

#endif /* SLEW_RATE_LIMITER_H */
