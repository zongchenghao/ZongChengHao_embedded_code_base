#include "Chassis_steer_task.h"
#include "Chassis_steer_app.h"
#include "Chassis_steer_function.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "Gimbal_steer_app.h"

extern ChassisHandle_t ChassisHandle;
extern GimbalHandle_t  gimbal_handle;

static void ChassisSensorUpdata(void);
void ChassisCtrlModeSwitch(void);
static void ChassisStopMode(void);
static void ChassisTestMode(void);
static void ChassisSeparateGimbalMode(void);
static void ChassisSpinMode(void);
static void ChassisFollowGimbalMode(void);

void Chassis_Task(void const * argument)
{
  for(;;)
  {
		ChassisSensorUpdata();
		ChassisCtrlModeSwitch();
		switch(ChassisHandle.ctrl_mode)
		{
			case CHASSIS_SEPARATE_GIMBAL:
			{
				ChassisSeparateGimbalMode();
			}
			break;
			
			case CHASSIS_FOLLOW_GIMBAL:
			{
				ChassisFollowGimbalMode();
			}
			break;
			
			case CHASSIS_SPIN:
			{
				ChassisSpinMode();
			}
			break;
			
			case CHASSIS_TEST:
			{
				ChassisTestMode();
			}
			break;
			
			case CHASSIS_STOP:
			{
				ChassisStopMode();
			}
			break;
			
			default:
				break;
			
		}
		
		Steer_Chassis_ControlCalc(&ChassisHandle);
		
		for (uint8_t i = 0; i < 4; i++)             //PID控制
    {
      ChassisHandle.chassis_motor[i].given_speed = ChassisHandle.wheel_rpm[i];
      ChassisHandle.chassis_motor[i].current_set = pid_calc(&ChassisHandle.chassis_motor[i].pid,
                                                             ChassisHandle.chassis_motor[i].motor_info->speed_rpm,
                                                             ChassisHandle.chassis_motor[i].given_speed);
			
			ChassisHandle.chassis_steer_motor[i].given_value=ChassisHandle.steeringAngleTarget[i];
			ChassisHandle.chassis_steer_motor[i].current_set = Chassis_Steer_PID_Calc(&ChassisHandle.chassis_steer_motor->pid, 
			                                                                          ChassisHandle.steeringAngleTarget[i],
			                                                                          ChassisHandle.steeringAngle[i]*-1,
			                                                                          ChassisHandle.chassis_steer_motor[i].sensor.palstance);//ChassisHandle.imu->gyro[2]*RAD_TO_ANGLE
    }                                                                                                                    //ChassisHandle.chassis_steer_motor->motor_info->speed_rpm
		
		if(ChassisHandle.ctrl_mode == CHASSIS_RELAX)     
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                ChassisHandle.chassis_motor[i].current_set = 0;
						  	ChassisHandle.chassis_steer_motor[i].current_set = 0;
            }
        }
				
		 Motor_SendMessage(&hcan2,MOTOR_1TO4_CONTROL_STD_ID,ChassisHandle.chassis_motor[0].current_set,
                                ChassisHandle.chassis_motor[1].current_set,
                                ChassisHandle.chassis_motor[2].current_set,
                                ChassisHandle.chassis_motor[3].current_set);       //轮输出
				
	  Motor_SendMessage(&hcan2,MOTOR_5TO8_CONTROL_STD_ID,ChassisHandle.chassis_steer_motor[0].current_set, 
									ChassisHandle.chassis_steer_motor[1].current_set, 
									ChassisHandle.chassis_steer_motor[2].current_set,
									ChassisHandle.chassis_steer_motor[3].current_set);   //舵输出
		
    osDelay(10);
  }
	
}

static void ChassisSensorUpdata(void)    //传感器数据更新
{
	 for(uint8_t i=0;i<4;i++)
	  {
		   ChassisHandle.chassis_steer_motor[i].sensor.palstance=ChassisHandle.chassis_steer_motor[i].motor_info->speed_rpm*6;	//更新四个电机的角速度值
		}
		
	ChassisHandle.gimbal_yaw_ecd_angle = gimbal_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.yaw_motor.motor_info->ecd,
                                                      			gimbal_handle.yaw_motor.offset_ecd.positive_offset_ecd);
		
	ChassisHandle.chassis_pitch=ChassisHandle.imu->attitude.pitch;
	ChassisHandle.chassis_roll=ChassisHandle.imu->attitude.roll;
	ChassisHandle.chassis_yaw=ChassisHandle.imu->attitude.yaw;
}

void ChassisCtrlModeSwitch(void)
{
	if(ChassisHandle.console->chassis_cmd == CHASSIS_RELEASE_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_RELAX;
	}
	
	else if(ChassisHandle.console->chassis_cmd == CHASSIS_FOLLOW_GIMBAL_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
	}
	
	else if(ChassisHandle.console->chassis_cmd == CHASSIS_SEPARATE_GIMBAL_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_SEPARATE_GIMBAL;
	}
	
	else if(ChassisHandle.console->chassis_cmd == CHASSIS_SPIN_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_SPIN;
	}
	
	else if(ChassisHandle.console->chassis_cmd == CHASSIS_TEST_CMD)
	{
		ChassisHandle.ctrl_mode = CHASSIS_TEST;
	}
	
	else if (ChassisHandle.console->chassis_cmd == CHASSIS_STOP_CMD)
  {
    ChassisHandle.ctrl_mode = CHASSIS_STOP;
	}
}
	

static void ChassisStopMode(void)//停止模式
{
   ChassisHandle.vx=0;
	 ChassisHandle.vy=0;
	 ChassisHandle.vw=0;
}

static void ChassisTestMode(void)//底盘测试模式VX,VY,VW测试
{
   ChassisHandle.vx=ChassisHandle.console->chassis.vx;
	 ChassisHandle.vy=ChassisHandle.console->chassis.vy;
	 ChassisHandle.vw=ChassisHandle.console->chassis.vw;
}
	
static void ChassisSeparateGimbalMode(void)//分离模式
{
	
	 ChassisHandle.vx = ChassisHandle.console->chassis.vx;
	 ChassisHandle.vy = ChassisHandle.console->chassis.vy;
	 ChassisHandle.vw=ChassisHandle.console->chassis.vw;
}	

static void ChassisSpinMode(void)//小陀螺模式
{
	
	ChassisHandle.vw = 400;//160为初始值（详细可见infantry_console.c中的小陀螺模式切换，会在160的基础上再乘以spin_rate）
	/*在除keyboard2模式下，vw的值在infantry_console.c中都没有过赋值，因此在此处自己赋值*/
  ChassisHandle.vx = ChassisHandle.console->chassis.vx;
	ChassisHandle.vy = ChassisHandle.console->chassis.vy;	
}

static void ChassisFollowGimbalMode(void)//跟随云台模式
{   
    ChassisHandle.vx = ChassisHandle.console->chassis.vx;
    ChassisHandle.vy = ChassisHandle.console->chassis.vy;
    ChassisHandle.vw = pid_calc(&ChassisHandle.chassis_follow_pid,
                                 ChassisHandle.gimbal_yaw_ecd_angle,0);   
}

