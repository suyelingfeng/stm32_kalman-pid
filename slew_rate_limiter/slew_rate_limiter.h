#ifndef SLEW_RATE_LIMITER_H
#define SLEW_RATE_LIMITER_H

#ifdef __cplusplus
extern "C" {
#endif

/** 分别限制输出上升和下降速度的执行器保护器。 */
typedef struct {
    float rise_rate; /**< 每秒允许的最大上升量，非负。 */
    float fall_rate; /**< 每秒允许的最大下降量，非负。 */
    float output;    /**< 最近一次限制后的输出。 */
} SlewRateLimiter;

/**
 * @brief 初始化斜率限制器。
 * @param rise_rate 每秒最大上升量；负值会转换为绝对值。
 * @param fall_rate 每秒最大下降量；负值会转换为绝对值。
 * @param initial_output 初始输出。
 */
void SlewRateLimiter_Init(SlewRateLimiter *limiter,
                          float rise_rate,
                          float fall_rate,
                          float initial_output);

/**
 * @brief 在给定时间步内向目标输出移动。
 * @param target 未限制的目标输出。
 * @param dt_seconds 更新周期，单位秒；非正值时保持上次输出。
 * @return 斜率限制后的输出。
 */
float SlewRateLimiter_Update(SlewRateLimiter *limiter,
                             float target,
                             float dt_seconds);

/** @brief 重置当前输出但保留上升、下降速率。 */
void SlewRateLimiter_Reset(SlewRateLimiter *limiter, float output);

#ifdef __cplusplus
}
#endif

#endif /* SLEW_RATE_LIMITER_H */