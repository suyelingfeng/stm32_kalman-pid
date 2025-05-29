/**
 * @file pid.c
 * @brief pid实现
 * @author suye
 * @date 2025/5/29
 */
#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"

typedef enum {
    PID_POSITION,  // 位置式PID
    PID_INCREMENT  // 增量式PID
} PID_Mode;

typedef struct {
    float Kp;       // 比例系数
    float Ki;       // 积分系数
    float Kd;       // 微分系数
    
    float max_out;  // 输出限幅
    float max_iout; // 积分限幅
    
    float set;      // 设定值
    float fdb;      // 反馈值
    
    float out;      // 输出值
    float Pout;     // 比例输出
    float Iout;     // 积分输出
    float Dout;     // 微分输出
    
    float last_err; // 上次误差
    float last_last_err; // 上上次误差
    
    PID_Mode mode;  // PID模式
} PID_Controller;

// PID初始化
void PID_Init(PID_Controller *pid, PID_Mode mode, 
              float Kp, float Ki, float Kd,
              float max_out, float max_iout);

// 位置式PID计算
float PID_Position_Calc(PID_Controller *pid, float set, float fdb);

// 增量式PID计算
float PID_Increment_Calc(PID_Controller *pid, float set, float fdb);

// PID重置
void PID_Reset(PID_Controller *pid);

#endif /* __PID_H */
