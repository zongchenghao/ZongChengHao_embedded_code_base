#include "Gimbal_steer_app.h"

GimbalHandle_t  gimbal_handle;

void GimbalAppConfig(void)
{
	gimbal_handle.console = Console_Pointer();
	gimbal_handle.imu = IMU_GetDataPointer();
	
	gimbal_handle.ctrl_mode = GIMBAL_INIT;
	gimbal_handle.pitch_motor.motor_info = Gimbal_pitch_Motor_Pointer();
  gimbal_handle.yaw_motor.motor_info = Gimbal_yaw_Motor_Pointer();
	
	gimbal_handle.yaw_motor.offset_ecd.positive_offset_ecd = 600; //根据安装方向自行调整
	gimbal_handle.pitch_motor.offset_ecd.positive_offset_ecd =6130; //根据安装方向自行调整
	
	gimbal_handle.yaw_motor.ecd_ratio = YAW_MOTO_POSITIVE_DIR * YAW_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
  gimbal_handle.pitch_motor.ecd_ratio = PITCH_MOTO_POSITIVE_DIR * PITCH_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
	
	gimbal_handle.yaw_motor.max_relative_angle = 90;//90
  gimbal_handle.yaw_motor.min_relative_angle = -90;//-90
  gimbal_handle.pitch_motor.max_relative_angle = 23;
  gimbal_handle.pitch_motor.min_relative_angle = -23;
	
//	pid_init(&GimbalHandle.yaw_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
//             60.0f, 0.0f, 40.0f);  //20
//  pid_init(&GimbalHandle.yaw_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
//             60.0f, 0.1f, 25.0f); //100
						 
	pid_init(&gimbal_handle.yaw_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             15.0f, 0.0f, 0.0f);  //20
  pid_init(&gimbal_handle.yaw_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
             15.0f, 0.0f, 0.0f); //100
	
  pid_init(&gimbal_handle.pitch_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             55.0f, 0.1f, 0.0f);// 60.0f, 0.1f, 0.0f
//  pid_init(&GimbalHandle.pitch_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 6000.0f,
//             75.0f, 0.1f, 15.0f); //70.0f, 0.0f, 15.0f
	pid_init(&gimbal_handle.pitch_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 6000.0f,
             20.0f, 0.1f, 0.0f); //70.0f, 0.0f, 15.0f
}



