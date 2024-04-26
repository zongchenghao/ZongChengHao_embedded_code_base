#ifndef __PID_H__
#define __PID_H__

#include "main.h"

enum
{
    LLAST = 0, //前次的数据值
    LAST, //上次的数据值
    NOW, //当前时刻的数据值
    POSITION_PID, //位置型PID
    DELTA_PID, //增量型PID
};//枚举从LLAST=0开始往下依次递增

typedef struct pid_t
{
    float p; //kp比例
    float i; //ki积分
    float d; //kd微分

    float set; //设定值
    float get; //获得值
    float err[3]; //误差值

    float pout; //比例输出
    float iout; //积分输出
    float dout; //微分输出
    float out; //总输出

    float input_max_err;    //input max err;
    float output_deadband;  //output deadband;

    int pid_mode; //pid模式
    float max_out; //输出限幅
    float integral_limit; //积分限幅

} pid_t;

typedef struct
{
    pid_t           outer_pid;
    pid_t           inter_pid;
    float           outer_ref;
    float           outer_fdb;
    float           inter_ref;
    float           inter_fdb;
} Double_PID_t;//双环PID结构体

float pid_calc(pid_t *pid, float get, float set);
void pid_clear(pid_t *pid);
void pid_init(pid_t*   pid, int mode, float maxout, float intergral_limit, float kp, float ki, float kd);
float DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb);
void Chassis_pid_init(void);

#endif
