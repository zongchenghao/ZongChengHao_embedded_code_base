#include "Chassis_steer_app.h"
#include "string.h"
#include "Gimbal_steer_app.h"
#include "Shoot_app.h"

ChassisHandle_t ChassisHandle;
extern GimbalHandle_t  gimbal_handle;
extern ShootHandle_t shoot_handle;

void Chassis_steerAppConfig(void)
{
	ChassisHandle.console=Console_Pointer();
	ChassisHandle.imu=IMU_GetDataPointer();
	ChassisHandle.ctrl_mode = CHASSIS_RELAX;
	
	ChassisHandle.structure.wheel_perimeter = WHEEL_PERIMETER;       //轮子硬性参数
  ChassisHandle.structure.wheeltrack = WHEELTRACK;
  ChassisHandle.structure.wheelbase = WHEELBASE;
	ChassisHandle.structure.Radius = RADIUS;
	
	ChassisHandle.chassis_steer_motor[0].offset_ecd.chassis_offset_ecd =1726;
	ChassisHandle.chassis_steer_motor[1].offset_ecd.chassis_offset_ecd =20;
	ChassisHandle.chassis_steer_motor[2].offset_ecd.chassis_offset_ecd =7789;
	ChassisHandle.chassis_steer_motor[3].offset_ecd.chassis_offset_ecd =3370;
	
	ChassisHandle.steer_set[0].speed_direction=-1;                         
	ChassisHandle.steer_set[1].speed_direction=-1;
	ChassisHandle.steer_set[2].speed_direction=1;
	ChassisHandle.steer_set[3].speed_direction=1;
	
	for (uint8_t i=0; i<4; i++)      //底盘8电机PID控制
	{ 
    ChassisHandle.chassis_motor[i].motor_info = ChassisMotor_Pointer(i);
		ChassisHandle.chassis_steer_motor[i].motor_info=Chassis_steer_Motor_Pointer(i);
		ChassisHandle.chassis_steer_motor[i].ecd_ratio=STEER_MOTO_POSITIVE_DIR * STEER_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;// 初始化舵机的转向系数
	  ChassisHandle.chassis_steer_motor[i].max_relative_angle = 180;
    ChassisHandle.chassis_steer_motor[i].min_relative_angle = -180;
		
		pid_init(&ChassisHandle.chassis_motor[i].pid, POSITION_PID, M3508_MOTOR_MAX_CURRENT, 3000.0f,
                 10.0f, 0.0f, 2.0f);
//		pid_init(&ChassisHandle.chassis_steer_motor[i].pid.outer_pid, POSITION_PID, 500.0f, 0.0f,
//                 20.0f, 0.0f, 4.0f);         // 20 0 4
//		pid_init(&ChassisHandle.chassis_steer_motor[i].pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
//                 40.0f, 0.1f, 0.0f);      // 40   
		pid_init(&ChassisHandle.chassis_steer_motor[i].pid.outer_pid, POSITION_PID, 500.0f, 0.0f,
                 20.0f, 0.0f, 0.0f);         // 20 0 4
		pid_init(&ChassisHandle.chassis_steer_motor[i].pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
                 10.0f, 0.0f, 0.0f);      // 40  
		
	}
	pid_init(&ChassisHandle.chassis_follow_pid, POSITION_PID, 300.0f, 50.0f, 5.0f, 0.0f, 0.0f);  //底盘跟随PID控制 3.5 0.01 4.5 8 0.027 3.5
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
	/*CAN2接收*/
  if(hcan->Instance == CAN2)
  {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  switch(rx_header.StdId)
	{
		/*行进轮*/
		case CHASSIS_MOTOR_LF_MESSAGE_ID :
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[0].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_MOTOR_RF_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[1].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_MOTOR_LB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[2].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_MOTOR_RB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_motor[3].motor_info, rx_data);
		break;
	}
		
	/*转向轮*/
	  case CHASSIS_STEER_MOTOR_LF_MESSAGE_ID :
	{
    Motor_EncoderData(ChassisHandle.chassis_steer_motor[0].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_STEER_MOTOR_RF_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_steer_motor[1].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_STEER_MOTOR_LB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_steer_motor[2].motor_info, rx_data);
		break;
	}
	
		case CHASSIS_STEER_MOTOR_RB_MESSAGE_ID:
	{
    Motor_EncoderData(ChassisHandle.chassis_steer_motor[3].motor_info, rx_data);
		break;
	}
		
	}
  }
	/*CAN1接收*/
	else if(hcan->Instance == CAN1)
  {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  switch(rx_header.StdId)
	{
	  case GIMBAL_PITCH_MOTOR :
	{
    Motor_EncoderData(gimbal_handle.pitch_motor.motor_info, rx_data);
		break;
	}
	
		case GIMBAL_YAW_MOTOR :
	{
    Motor_EncoderData(gimbal_handle.yaw_motor.motor_info, rx_data);
		break;
	}
	
	 case FRICTION_WHEEL_1_MESSAGE_ID :
	{
    Motor_EncoderData(shoot_handle.fric_wheel_motor[0].motor_info, rx_data);
		break;
	}
	
	case FRICTION_WHEEL_2_MESSAGE_ID :
	{
    Motor_EncoderData(shoot_handle.fric_wheel_motor[1].motor_info, rx_data);
		break;
	}
	
	case TRIGGER_MOTOR_MESSAGE_ID :
	{
    Motor_EncoderData(shoot_handle.trigger_motor.motor_info, rx_data);
		break;
	}
	
//	case MAGAZINE_MOTOR_MESSAGE_ID:
//	{
//		Motor_EncoderData(shoot_handle.magazine_motor.motor_info, rx_data);
//		break;
//	}
	
	}
  }
}

void calculateRoundCnt(ChassisHandle_t* chassis_handle)    
{
	static float last_encoder[4] = {0};
	for(uint8_t i=0;i<4;i++)
	{
		float now_encoder = chassis_handle->chassis_steer_motor[i].motor_info->ecd
						  -	chassis_handle->chassis_steer_motor[i].offset_ecd.chassis_offset_ecd;
		now_encoder = now_encoder/8192*360;
		
		if(now_encoder - last_encoder[i] > 180)
			chassis_handle->motor_circle[i] --;
		else if(now_encoder - last_encoder[i] < -180) 
			chassis_handle->motor_circle[i] ++;
		
		last_encoder[i] = now_encoder;
		chassis_handle->steeringAngle[i] = -now_encoder -  chassis_handle->motor_circle[i]*360;
		chassis_handle->chassis_steer_motor[i].sensor.relative_angle=chassis_handle->steeringAngle[i]*-1;
	}
}

void calculateTargetRoundCnt(ChassisHandle_t* chassis_handle)   
{
	float now_target[4];
	static float last_target[4]={0,0,0,0};
	memcpy(now_target, chassis_handle->steeringAngleTarget, 4 * sizeof(fp32));
	
	for(uint8_t i=0;i<4;i++)
	{
		if(now_target[i] - last_target[i] > 180) 
			chassis_handle->motor_target_count[i] --;
		else if(now_target[i] - last_target[i]  < -180) 
			chassis_handle->motor_target_count[i] ++;
		
		last_target[i] = now_target[i];
		chassis_handle->steeringAngleTarget[i] = now_target[i] + chassis_handle->motor_target_count[i]*360 ;
	}
}
