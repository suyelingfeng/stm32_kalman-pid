#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/** PID 离散实现模式。 */
typedef enum {
    PID_POSITION = 0, /**< 位置式：直接计算完整输出。 */
    PID_INCREMENT    /**< 增量式：在上次输出上累加本周期增量。 */
} PID_Mode;

/** 位置式和增量式 PID 的参数与运行状态。 */
typedef struct {
    float Kp;            /**< 比例系数。 */
    float Ki;            /**< 每采样周期积分系数。 */
    float Kd;            /**< 每采样周期微分系数。 */
    float max_out;       /**< 输出绝对值上限。 */
    float max_iout;      /**< 位置式积分项绝对值上限。 */
    float set;           /**< 最近一次设定值。 */
    float fdb;           /**< 最近一次反馈值。 */
    float out;           /**< 最近一次限幅后的最终输出。 */
    float Pout;          /**< 最近一次比例分量或比例增量。 */
    float Iout;          /**< 最近一次积分分量或积分增量。 */
    float Dout;          /**< 最近一次微分分量或微分增量。 */
    float last_err;      /**< 上一次误差 e(k-1)。 */
    float last_last_err; /**< 上上次误差 e(k-2)。 */
    PID_Mode mode;       /**< PID_POSITION 或 PID_INCREMENT。 */
} PID_Controller;

/**
 * @brief 初始化 PID 参数并清空运行状态。
 * @param max_out 输出绝对值上限；负值会转换为绝对值。
 * @param max_iout 位置式积分项绝对值上限。
 * @note Ki、Kd 默认已经包含固定采样周期的影响。
 */
void PID_Init(PID_Controller *pid, PID_Mode mode,
              float Kp, float Ki, float Kd,
              float max_out, float max_iout);

/** @brief 计算位置式 PID，并返回限幅后的最终输出。 */
float PID_Position_Calc(PID_Controller *pid, float set, float fdb);

/**
 * @brief 计算增量式 PID，更新 pid->out。
 * @return 本周期未限幅的输出增量；最终限幅输出保存在 pid->out。
 */
float PID_Increment_Calc(PID_Controller *pid, float set, float fdb);

/** @brief 按 pid->mode 分派计算，并始终返回限幅后的最终输出。 */
float PID_Calc(PID_Controller *pid, float set, float fdb);

/** @brief 清空误差、分量和输出状态，但保留参数与模式。 */
void PID_Reset(PID_Controller *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */