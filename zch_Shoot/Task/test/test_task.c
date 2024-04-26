#include "test_task.h"
#include "cmsis_os.h"
#include "motor.h"
#include "bsp_can.h"
#include "pid.h"
#include "remote_control.h"
#include "Gimbal_steer_app.h"
#include "Gimbal_steer_function.h"
#include "math.h"

//extern GimbalHandle_t GimbalHandle;

extern MotorInfo_t chassis_motor[4]; //底盘驱动电机
extern MotorInfo_t chassis_steer_motor[4]; //底盘转向电机
extern RC_Info_t m_rc_info;

fp32 yaw_target=0;

void TestTask(void const * argument)
{
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
//		GimbalHandle.pitch_motor.given_value = 500;
//    GimbalHandle.pitch_motor.current_set = pid_calc(&GimbalHandle.pitch_motor.pid.inter_pid,
//                                                             GimbalHandle.pitch_motor.motor_info->speed_rpm,
//                                                             GimbalHandle.pitch_motor.given_value);
//		
//		GimbalHandle.yaw_motor.sensor.relative_angle =  GimbalHandle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(GimbalHandle.yaw_motor.motor_info->ecd,
//                                                      			GimbalHandle.yaw_motor.offset_ecd.positive_offset_ecd);
//		
//		GimbalHandle.yaw_motor.sensor.gyro_angle = GimbalHandle.imu->attitude.yaw;
//		
//		yaw_target=GimbalHandle.yaw_motor.sensor.gyro_angle+GimbalHandle.console->gimbal.yaw_v*50;
//		GimbalHandle.yaw_motor.given_value = AngleTransform(yaw_target, GimbalHandle.yaw_motor.sensor.gyro_angle);
//		
////		GimbalHandle.yaw_motor.current_set=Gimbal_PID_Calc(&GimbalHandle.yaw_motor.pid, GimbalHandle.yaw_motor.given_value, GimbalHandle.yaw_motor.sensor.relative_angle,GimbalHandle.yaw_motor.motor_info->speed_rpm);
//		GimbalHandle.yaw_motor.current_set=Gimbal_PID_Calc(&GimbalHandle.yaw_motor.pid, GimbalHandle.yaw_motor.given_value, GimbalHandle.yaw_motor.sensor.gyro_angle,GimbalHandle.yaw_motor.motor_info->speed_rpm);
//		
//		
////		GimbalHandle.yaw_motor.given_value = 1000;
////    GimbalHandle.yaw_motor.current_set = pid_calc(&GimbalHandle.yaw_motor.pid.inter_pid,
////                                                             GimbalHandle.yaw_motor.motor_info->speed_rpm,
////                                                             GimbalHandle.yaw_motor.given_value);
//		
//		Motor_SendMessage(&hcan1,MOTOR_5TO8_CONTROL_STD_ID,GimbalHandle.pitch_motor.current_set,GimbalHandle.yaw_motor.current_set,0,0);
		
//		pid_calc(&chassis_steer_motor_outer_pid[0],chassis_steer_motor[0].ecd/ENCODER_ANGLE_RATIO,360);
//		chassis_steer_motor[0].given_current=pid_calc(&chassis_steer_motor_inter_pid[0],chassis_steer_motor[0].speed_rpm,chassis_steer_motor_outer_pid[0].out);

    osDelay(40);
  }
}




