#include "pid.h"

#include <stddef.h>

static float PID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PID_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void PID_Init(PID_Controller *pid, PID_Mode mode,
              float Kp, float Ki, float Kd,
              float max_out, float max_iout)
{
    if (pid == NULL) {
        return;
    }
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->max_out = PID_Abs(max_out);
    pid->max_iout = PID_Abs(max_iout);
    pid->mode = mode;
    PID_Reset(pid);
}

float PID_Position_Calc(PID_Controller *pid, float set, float fdb)
{
    float err;
    if (pid == NULL) {
        return 0.0f;
    }
    pid->set = set;
    pid->fdb = fdb;
    err = set - fdb;
    /* 位置式 PID：积分项单独限幅，减轻积分饱和。 */
    pid->Pout = pid->Kp * err;
    pid->Iout = PID_Clamp(pid->Iout + pid->Ki * err, pid->max_iout);
    pid->Dout = pid->Kd * (err - pid->last_err);
    pid->last_last_err = pid->last_err;
    pid->last_err = err;
    pid->out = PID_Clamp(pid->Pout + pid->Iout + pid->Dout,
                         pid->max_out);
    return pid->out;
}

float PID_Increment_Calc(PID_Controller *pid, float set, float fdb)
{
    float err;
    float increment;
    if (pid == NULL) {
        return 0.0f;
    }
    pid->set = set;
    pid->fdb = fdb;
    err = set - fdb;
    /* 增量式 PID 使用最近三次误差计算本周期输出增量。 */
    pid->Pout = pid->Kp * (err - pid->last_err);
    pid->Iout = pid->Ki * err;
    pid->Dout = pid->Kd *
                (err - 2.0f * pid->last_err + pid->last_last_err);
    increment = pid->Pout + pid->Iout + pid->Dout;
    pid->last_last_err = pid->last_err;
    pid->last_err = err;
    pid->out = PID_Clamp(pid->out + increment, pid->max_out);
    return increment;
}

float PID_Calc(PID_Controller *pid, float set, float fdb)
{
    if (pid == NULL) {
        return 0.0f;
    }
    /* 统一入口始终返回最终输出，而非增量式的原始增量。 */
    if (pid->mode == PID_INCREMENT) {
        (void)PID_Increment_Calc(pid, set, fdb);
        return pid->out;
    }
    return PID_Position_Calc(pid, set, fdb);
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->out = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->last_err = 0.0f;
    pid->last_last_err = 0.0f;
}