/**
 * @file pid.c
 * @brief pid实现
 * @author suye
 * @date 2025/5/29
 */
#include "pid.h"
#include "stm32f1xx_hal.h"
#include"include.h"
/**
 * @brief PID控制器初始化
 * @param pid PID控制器指针
 * @param mode PID模式(PID_POSITION/PID_INCREMENT)
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param max_out 输出限幅值
 * @param max_iout 积分限幅值
 */
void PID_Init(PID_Controller *pid, PID_Mode mode, 
              float Kp, float Ki, float Kd,
              float max_out, float max_iout) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->mode = mode;
    PID_Reset(pid);
}

/**
 * @brief 位置式PID计算
 * @param pid PID控制器指针
 * @param set 设定值
 * @param fdb 反馈值
 * @return PID输出值
 */
float PID_Position_Calc(PID_Controller *pid, float set, float fdb) {
    pid->set = set;
    pid->fdb = fdb;
    float err = pid->set - pid->fdb;
    
    // 比例项
    pid->Pout = pid->Kp * err;
    
    // 积分项
    pid->Iout += pid->Ki * err;
    // 积分限幅
    if(pid->Iout > pid->max_iout) pid->Iout = pid->max_iout;
    else if(pid->Iout < -pid->max_iout) pid->Iout = -pid->max_iout;
    
    // 微分项
    pid->Dout = pid->Kd * (err - pid->last_err);
    pid->last_err = err;
    
    // 输出总和
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    
    // 输出限幅
    if(pid->out > pid->max_out) pid->out = pid->max_out;
    else if(pid->out < -pid->max_out) pid->out = -pid->max_out;
    
    return pid->out;
}

/**
 * @brief 增量式PID计算
 * @param pid PID控制器指针
 * @param set 设定值
 * @param fdb 反馈值
 * @return PID输出增量值
 */
float PID_Increment_Calc(PID_Controller *pid, float set, float fdb) {
    pid->set = set;
    pid->fdb = fdb;
    float err = pid->set - pid->fdb;
    
    // 计算增量
    float increment = 0;
    increment += pid->Kp * (err - pid->last_err); // P
    increment += pid->Ki * err;                   // I
    increment += pid->Kd * (err - 2*pid->last_err + pid->last_last_err); // D
    
    // 保存误差记录
    pid->last_last_err = pid->last_err;
    pid->last_err = err;
    
    // 计算输出
    pid->out += increment;
    
    // 输出限幅
    if(pid->out > pid->max_out) pid->out = pid->max_out;
    else if(pid->out < -pid->max_out) pid->out = -pid->max_out;
    
    return increment;
}

/**
 * @brief PID重置
 * @param pid PID控制器指针
 */
void PID_Reset(PID_Controller *pid) {
    pid->set = 0;
    pid->fdb = 0;
    pid->out = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->last_err = 0;
    pid->last_last_err = 0;
}
// 位置式PID使用
//PID_Controller pos_pid;
// PID_Init(&pos_pid, PID_POSITION, 1.0, 0.1, 0.05, 1000, 500);

// float output = PID_Position_Calc(&pos_pid, target_value, current_value);

// // 增量式PID使用  
// PID_Controller inc_pid;
// PID_Init(&inc_pid, PID_INCREMENT, 0.8, 0.05, 0.03, 800, 300);

// float increment = PID_Increment_Calc(&inc_pid, target_value, current_value);

// // 重置PID控制器
// PID_Reset(&pos_pid);