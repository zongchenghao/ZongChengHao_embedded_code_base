#ifndef __CHASSIS_STEER_FUNCTION_H__
#define __CHASSIS_STEER_FUNCTION_H__

#include "main.h"
#include "pid.h"
#include "struct_typedef.h"
#include "Chassis_steer_app.h"

fp32 Chassis_Steer_PID_Calc(Chassis_steer_pid_t* pid, fp32 angle_ref, fp32 angle_fdb, fp32 speed_fdb);
void Steer_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw);  //∂Ê¬÷µ◊≈ÃΩ‚À„
void Steer_Chassis_ControlCalc(ChassisHandle_t* chassis_handle);

#endif
