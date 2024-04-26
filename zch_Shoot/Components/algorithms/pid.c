#include "pid.h"
#include "stdio.h"
#include "math.h"
#include "motor.h"

/*绝对值限制*/
static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/*pid初始化，将数据写入pid结构体*/
static void pid_param_init(
    pid_t*   pid,
    int   mode,
    float maxout,
    float intergral_limit,
    float    kp,
    float    ki,
    float    kd)
{

    pid->integral_limit = intergral_limit;
    pid->max_out        = maxout;
    pid->pid_mode       = mode;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

}

static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out  = 0;
}

/*pid计算*/
float pid_calc(pid_t *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err[NOW] = set - get;

    if ((pid->input_max_err != 0) && (fabs(pid->err[NOW]) > pid->input_max_err))
        return 0;

    if (pid->pid_mode == POSITION_PID) //position PID
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

        abs_limit(&(pid->iout), pid->integral_limit);
        pid->out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }
    else if (pid->pid_mode == DELTA_PID) //delta PID
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        pid->out += pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST]  = pid->err[NOW];


    if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
        return 0;
    else
        return pid->out;
}

void pid_clear(pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->err[NOW] = pid->err[LAST] = pid->err[LLAST] = 0.0f;
    pid->out = pid->pout = pid->iout = pid->dout = 0.0f;
    pid->get = pid->set = 0.0f;
}

void pid_init(
    pid_t*   pid,
    int mode,
    float maxout,
    float intergral_limit,

    float kp,
    float ki,
    float kd)
{
    pid_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
    pid_reset(pid, kp, ki, kd);
}
/*双环PID解算*/
/*输入外环的设定值和当前值进行PID运算，外环的输出作为内环的输入，再进行设定值与当前值的pid运算，最终内环输出*/
float DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb)
{
    dpid->outer_ref = outer_ref;
    dpid->outer_fdb = outer_fdb;
    pid_calc(&dpid->outer_pid, dpid->outer_fdb, dpid->outer_ref);
    dpid->inter_ref = dpid->outer_pid.out;
    dpid->inter_fdb = inter_fdb;
    pid_calc(&dpid->inter_pid, dpid->inter_fdb, dpid->inter_ref);
    return dpid->inter_pid.out;
}
