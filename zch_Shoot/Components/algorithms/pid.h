#ifndef __PID_H__
#define __PID_H__

#include "main.h"

enum
{
    LLAST = 0, //ǰ�ε�����ֵ
    LAST, //�ϴε�����ֵ
    NOW, //��ǰʱ�̵�����ֵ
    POSITION_PID, //λ����PID
    DELTA_PID, //������PID
};//ö�ٴ�LLAST=0��ʼ�������ε���

typedef struct pid_t
{
    float p; //kp����
    float i; //ki����
    float d; //kd΢��

    float set; //�趨ֵ
    float get; //���ֵ
    float err[3]; //���ֵ

    float pout; //�������
    float iout; //�������
    float dout; //΢�����
    float out; //�����

    float input_max_err;    //input max err;
    float output_deadband;  //output deadband;

    int pid_mode; //pidģʽ
    float max_out; //����޷�
    float integral_limit; //�����޷�

} pid_t;

typedef struct
{
    pid_t           outer_pid;
    pid_t           inter_pid;
    float           outer_ref;
    float           outer_fdb;
    float           inter_ref;
    float           inter_fdb;
} Double_PID_t;//˫��PID�ṹ��

float pid_calc(pid_t *pid, float get, float set);
void pid_clear(pid_t *pid);
void pid_init(pid_t*   pid, int mode, float maxout, float intergral_limit, float kp, float ki, float kd);
float DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb);
void Chassis_pid_init(void);

#endif
