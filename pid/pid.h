#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PID_POSITION = 0,
    PID_INCREMENT
} PID_Mode;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float max_out;
    float max_iout;
    float set;
    float fdb;
    float out;
    float Pout;
    float Iout;
    float Dout;
    float last_err;
    float last_last_err;
    PID_Mode mode;
} PID_Controller;

void PID_Init(PID_Controller *pid, PID_Mode mode,
              float Kp, float Ki, float Kd,
              float max_out, float max_iout);
float PID_Position_Calc(PID_Controller *pid, float set, float fdb);
/* Updates pid->out and returns this cycle's output increment. */
float PID_Increment_Calc(PID_Controller *pid, float set, float fdb);
/* Dispatches by pid->mode and always returns the final, limited output. */
float PID_Calc(PID_Controller *pid, float set, float fdb);
void PID_Reset(PID_Controller *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */