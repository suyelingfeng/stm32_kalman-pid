#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 带回差的直接作用开关控制器，适用于加热、充液等场景。 */
typedef struct {
    float setpoint;      /**< 目标值。 */
    float half_band;     /**< 设定点两侧的半回差宽度，始终为非负值。 */
    float output_low;    /**< 低状态输出。 */
    float output_high;   /**< 高状态输出。 */
    uint8_t high_active; /**< 非零表示当前保持高输出。 */
} HysteresisController;

/** 二状态线性反馈控制器：u = Kr*r - K1*x1 - K2*x2。 */
typedef struct {
    float k1;             /**< 状态 1 反馈增益。 */
    float k2;             /**< 状态 2 反馈增益。 */
    float reference_gain; /**< 参考输入前馈增益 Kr。 */
    float max_output;     /**< 输出绝对值上限。 */
    float output;         /**< 最近一次限幅后的输出。 */
} StateFeedback2Controller;

/** 一阶线性 ADRC：二阶 ESO 加扰动补偿控制律。 */
typedef struct {
    float observer_bandwidth;   /**< ESO 带宽，单位 rad/s。 */
    float controller_bandwidth; /**< 控制带宽，单位 rad/s。 */
    float plant_gain;           /**< 被控对象输入增益估计 b0，符号必须正确。 */
    float max_output;           /**< 输出绝对值上限。 */
    float z1;                   /**< ESO 对系统输出的估计。 */
    float z2;                   /**< ESO 对总扰动的估计。 */
    float output;               /**< 最近一次控制输出。 */
} FirstOrderLadrc;

/**
 * @brief 初始化直接作用回差控制器。
 * @param controller 控制器实例。
 * @param setpoint 目标值。
 * @param half_band 单侧回差宽度；负值会转换为绝对值。
 * @param output_low 低状态输出。
 * @param output_high 高状态输出。
 * @param high_active 初始状态，非零为高输出。
 */
void HysteresisController_Init(HysteresisController *controller,
                               float setpoint,
                               float half_band,
                               float output_low,
                               float output_high,
                               uint8_t high_active);

/**
 * @brief 根据测量值更新回差状态。
 * @param controller 控制器实例。
 * @param measurement 当前测量值。
 * @return 当前高或低状态输出。
 * @note 低于 setpoint-half_band 切至高输出；高于
 *       setpoint+half_band 切至低输出；回差带内保持原状态。
 */
float HysteresisController_Update(HysteresisController *controller,
                                  float measurement);

/** @brief 在线修改回差控制器的目标值。 */
void HysteresisController_Setpoint(HysteresisController *controller,
                                   float setpoint);

/**
 * @brief 初始化二状态反馈控制器。
 * @param controller 控制器实例。
 * @param k1 状态 1 反馈增益。
 * @param k2 状态 2 反馈增益。
 * @param reference_gain 参考输入增益 Kr。
 * @param max_output 输出绝对值上限；负值会转换为绝对值。
 */
void StateFeedback2_Init(StateFeedback2Controller *controller,
                         float k1,
                         float k2,
                         float reference_gain,
                         float max_output);

/**
 * @brief 计算二状态反馈控制量。
 * @return 按 max_output 限幅后的控制输出。
 */
float StateFeedback2_Update(StateFeedback2Controller *controller,
                            float reference,
                            float state1,
                            float state2);

/** @brief 清零二状态反馈控制器的输出状态，不修改增益。 */
void StateFeedback2_Reset(StateFeedback2Controller *controller);

/**
 * @brief 初始化一阶线性 ADRC。
 * @param controller 控制器实例。
 * @param observer_bandwidth ESO 带宽，单位 rad/s。
 * @param controller_bandwidth 控制带宽，单位 rad/s。
 * @param plant_gain 输入增益估计 b0；接近零时回退为 1。
 * @param max_output 输出绝对值上限。
 * @param initial_measurement 初始测量值，用于无扰初始化 ESO。
 */
void FirstOrderLadrc_Init(FirstOrderLadrc *controller,
                          float observer_bandwidth,
                          float controller_bandwidth,
                          float plant_gain,
                          float max_output,
                          float initial_measurement);

/**
 * @brief 更新 ESO 并计算扰动补偿后的控制量。
 * @param reference 目标值。
 * @param measurement 当前测量值。
 * @param dt_seconds 固定控制周期，单位秒；非正值时保持上次输出。
 * @return 限幅后的控制输出。
 */
float FirstOrderLadrc_Update(FirstOrderLadrc *controller,
                            float reference,
                            float measurement,
                            float dt_seconds);

/** @brief 用当前测量值重置 ESO 和输出，不修改整定参数。 */
void FirstOrderLadrc_Reset(FirstOrderLadrc *controller,
                           float measurement);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */