#include "Gimbal_steer_task.h"
#include "Gimbal_steer_app.h"
#include "cmsis_os.h"
#include "Gimbal_steer_function.h"
#include "math.h"
#include "user_lib.h"
#include "bsp_can.h"

extern GimbalHandle_t gimbal_handle;

static void GimbalCtrlModeSwitch(void);
static void GimbalInitMode(void);
static void GimbalNormalMode(void);

void Gimbal_Steer_Task(void const * argument)
{
  for(;;)
  {
		GimbalCtrlModeSwitch();
		switch (gimbal_handle.ctrl_mode)
    {
       case GIMBAL_INIT:
       {
         GimbalInitMode();
       }break;
			 
       case GIMBAL_NORMAL:
       {
         GimbalNormalMode();
       }break;
						
       default:
         break;
    }
				
    if (gimbal_handle.ctrl_mode == GIMBAL_RELAX)
    {
       pid_clear(&gimbal_handle.yaw_motor.pid.outer_pid);
       pid_clear(&gimbal_handle.yaw_motor.pid.inter_pid);
       pid_clear(&gimbal_handle.pitch_motor.pid.outer_pid);
       pid_clear(&gimbal_handle.pitch_motor.pid.inter_pid);
       gimbal_handle.yaw_motor.current_set = 0;
       gimbal_handle.pitch_motor.current_set = 0;
    }
		
		Motor_SendMessage(&hcan1,MOTOR_5TO8_CONTROL_STD_ID,gimbal_handle.pitch_motor.current_set,gimbal_handle.yaw_motor.current_set,0,0);
		
    osDelay(10);
  }
	
}

static void GimbalCtrlModeSwitch(void)
{
	
    gimbal_handle.last_ctrl_mode = gimbal_handle.ctrl_mode;
    if (gimbal_handle.console->gimbal_cmd == GIMBAL_RELEASE_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_RELAX;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_INIT_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_INIT;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_NORMAL_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_NORMAL;
    }
   
}

static void GimbalInitMode(void)
{
	
	gimbal_handle.yaw_motor.mode = ENCONDE_MODE;
  gimbal_handle.pitch_motor.mode = ENCONDE_MODE;
	
	gimbal_handle.yaw_motor.given_value =  gimbal_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.yaw_motor.motor_info->ecd,
                                                      			gimbal_handle.yaw_motor.offset_ecd.positive_offset_ecd);
  gimbal_handle.pitch_motor.given_value = gimbal_handle.pitch_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.pitch_motor.motor_info->ecd,
                                                            gimbal_handle.pitch_motor.offset_ecd.positive_offset_ecd);
	
//	gimbal_handle.yaw_motor.given_value = AngleTransform(60.0, gimbal_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.yaw_motor.motor_info->ecd,
//                                                      	                                                  gimbal_handle.yaw_motor.offset_ecd.positive_offset_ecd));	

	gimbal_handle.yaw_motor.current_set = Gimbal_PID_Calc(&gimbal_handle.yaw_motor.pid, gimbal_handle.yaw_motor.given_value, 
	                                                      gimbal_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.yaw_motor.motor_info->ecd,
                                                      	                                                  gimbal_handle.yaw_motor.offset_ecd.positive_offset_ecd),
                                                       	gimbal_handle.imu->gyro[2] * RAD_TO_ANGLE);
	gimbal_handle.pitch_motor.current_set = Gimbal_PID_Calc(&gimbal_handle.pitch_motor.pid, gimbal_handle.pitch_motor.given_value, 
	                                                      gimbal_handle.pitch_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.pitch_motor.motor_info->ecd,
                                                      	                                                  gimbal_handle.pitch_motor.offset_ecd.positive_offset_ecd),
                                                       	gimbal_handle.imu->gyro[1] * RAD_TO_ANGLE);
	
//	if (fabsf(gimbal_handle.pitch_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.pitch_motor.motor_info->ecd,
//                                                      			gimbal_handle.pitch_motor.offset_ecd.positive_offset_ecd)) <= 2.0f)
//    {

//        if (fabsf(gimbal_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.yaw_motor.motor_info->ecd,
//                                                      			gimbal_handle.yaw_motor.offset_ecd.positive_offset_ecd)) <= 3.0f )
//        {
//            gimbal_handle.ctrl_mode = GIMBAL_NORMAL;
//        }
//    }
	
}

static void GimbalNormalMode(void)
{
	 fp32 yaw_target = 0;
   gimbal_handle.yaw_motor.mode = GYRO_MODE;
   gimbal_handle.pitch_motor.mode = ENCONDE_MODE;
	
	 gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
   gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
	
	 yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v*100;
	
	 gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.imu->attitude.yaw);
   gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
	 
	 VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);
	 
	 gimbal_handle.yaw_motor.current_set = Gimbal_PID_Calc(&gimbal_handle.yaw_motor.pid, gimbal_handle.yaw_motor.given_value, gimbal_handle.imu->attitude.yaw, gimbal_handle.imu->gyro[2] * RAD_TO_ANGLE);
	 gimbal_handle.pitch_motor.current_set = Gimbal_PID_Calc(&gimbal_handle.pitch_motor.pid, gimbal_handle.pitch_motor.given_value, 
	                                                      gimbal_handle.pitch_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.pitch_motor.motor_info->ecd,
                                                      	                                                  gimbal_handle.pitch_motor.offset_ecd.positive_offset_ecd),
                                                       	gimbal_handle.imu->gyro[1] * RAD_TO_ANGLE);

}







