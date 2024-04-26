#ifndef __GIMBAL_STEER_FUNCTION_H__
#define __GIMBAL_STEER_FUNCTION_H__

#include "main.h"
#include "struct_typedef.h"
#include "Gimbal_steer_app.h"


fp32 Gimbal_PID_Calc(Gimbal_PID_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb);
fp32 AngleTransform(fp32 target_angle, fp32 gyro_angle);

#endif
