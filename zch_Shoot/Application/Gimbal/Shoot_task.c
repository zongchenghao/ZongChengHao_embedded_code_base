#include "Shoot_app.h"
#include "Shoot_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "math.h"
#include "tim.h"

extern ShootHandle_t shoot_handle;

//ShootHandle_t magazine_handle;

static void ShootSensorUpdata(void);
static void ShootCtrlModeSwitch(void);
static void MagazineCtrlModeSwitch(void);
static void Shoot_TriggerMotorCtrl(ShootHandle_t* handle);
static void Shoot_MagazineMotorCtrl(ShootHandle_t* handle);
static void Shoot_FrictionWheelMotorCtrl(ShootCtrlMode_e mode, FrictionWheelMotor_t motor[2]);

void ShootTask(void const * argument)
{
  ShootTaskConfig();
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  for(;;)
  {
		ShootSensorUpdata();
		ShootCtrlModeSwitch();
		MagazineCtrlModeSwitch();
		
		Shoot_MagazineMotorCtrl(&shoot_handle);
		Shoot_TriggerMotorCtrl(&shoot_handle);
		Shoot_FrictionWheelMotorCtrl(shoot_handle.ctrl_mode, shoot_handle.fric_wheel_motor);
		
		if (shoot_handle.ctrl_mode == SHOOT_RELAX)
    {
        shoot_handle.fric_wheel_motor[0].current_set = 0;
        shoot_handle.fric_wheel_motor[1].current_set = 0;
        shoot_handle.trigger_motor.current_set = 0;
    }
		
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,shoot_handle.magazine_motor.GM996R_pwm);
		Motor_SendMessage(&hcan1, MOTOR_1TO4_CONTROL_STD_ID, shoot_handle.fric_wheel_motor[0].current_set, 
		                                                     shoot_handle.fric_wheel_motor[1].current_set,
		                                                     TRIGGER_MOTOR_POSITIVE_DIR*shoot_handle.trigger_motor.current_set,
		                                                     0); //CAN发送函数
		
    osDelay(SHOOT_TASK_PERIOD);
  }
}

static void ShootSensorUpdata(void)
{
	
//	shoot_handle.magazine_motor.speed = (fp32)shoot_handle.magazine_motor.motor_info->speed_rpm * MAGAZINE_MOTOR_REDUCTION_RATIO;
//  shoot_handle.magazine_motor.angle = shoot_handle.magazine_motor.ecd_ratio
//									 * (fp32)(shoot_handle.magazine_motor.motor_info->total_ecd - shoot_handle.magazine_motor.offset_ecd);
	
	shoot_handle.trigger_motor.speed = (fp32)shoot_handle.trigger_motor.motor_info->speed_rpm * TRIGGER_MOTOR_REDUCTION_RATIO * TRIGGER_MOTOR_POSITIVE_DIR;
	shoot_handle.trigger_motor.angle = shoot_handle.trigger_motor.ecd_ratio
									 * (fp32)(shoot_handle.trigger_motor.motor_info->total_ecd - shoot_handle.trigger_motor.offset_ecd);
	
}

static void ShootCtrlModeSwitch(void)
{
	
	 if (shoot_handle.console->shoot_cmd == SHOOT_RELEASE_CMD)
    {
        shoot_handle.ctrl_mode = SHOOT_RELAX;
    }
    else if (shoot_handle.console->shoot_cmd == SHOOT_START_CMD)
    {
        shoot_handle.ctrl_mode = SHOOT_START;
    }
    else if (shoot_handle.console->shoot_cmd == SHOOT_STOP_CMD)
    {
        shoot_handle.ctrl_mode = SHOOT_STOP;
    }
		
}

static void MagazineCtrlModeSwitch(void)
{
    if (shoot_handle.console->magazine_cmd == MAGAZINE_OFF_CMD)
    {
        shoot_handle.magazine_state = MAGAZINE_OFF_STATE;
    }
    else if (shoot_handle.console->magazine_cmd == MAGAZINE_ON_CMD)
    {
        shoot_handle.magazine_state = MAGAZINE_ON_STATE;
    }
}

static void Shoot_MagazineMotorCtrl(ShootHandle_t* handle)
{
//		static fp32 zero_angle = 20.0f;
		
		 if (handle->magazine_state != MAGAZINE_INIT_STATE)
     {
        if (handle->magazine_state == MAGAZINE_OFF_STATE)
        {
            handle->magazine_motor.GM996R_pwm = 20;
//						magazine_handle.magazine_state = MAGAZINE_OFF_STATE;
        }
        else
        {
            handle->magazine_motor.GM996R_pwm = 80;
//						magazine_handle.magazine_state = MAGAZINE_ON_STATE;
        }
//        handle->magazine_motor.current_set = DoublePID_Calc(&handle->magazine_motor.pid,
//                                                            handle->magazine_motor.set_angle,
//                                                            handle->magazine_motor.angle,
//                                                            handle->magazine_motor.speed);
				
		}
				
			else
		 {
				 handle->magazine_motor.GM996R_pwm = 0;
		 }
    
}


static void Shoot_TriggerMotorCtrl(ShootHandle_t* handle)
{
	
	static int8_t max_bullet_nums = 0;             //最大子弹数
	
	if (handle->ctrl_mode == SHOOT_START)           //开始射击模式
	{
		if (handle->trigger_state == TRIGGER_END)       //拨弹结束状态
    {
				if(handle->console->shoot.fire_cmd == ONE_FIRE_CMD)   //单发热量计算
				{
						max_bullet_nums = 75 / ONE_BULLET_HEAT - 4;
				}	
				else if(handle->console->shoot.fire_cmd == RAPID_FIRE_CMD)   //五连发热量计算
				{
						max_bullet_nums = 75/ ONE_BULLET_HEAT;
				}
	
				if (handle->console->shoot.fire_cmd == ONE_FIRE_CMD && max_bullet_nums >= 1)
				{
						handle->fire_bullet_number = 1;
				}
	
				else if (handle->console->shoot.fire_cmd == RAPID_FIRE_CMD && max_bullet_nums >= 5)
				{
						handle->fire_bullet_number = 5;
				}
	
				if (handle->fire_bullet_number != 0)
				{
						handle->trigger_state = TRIGGER_BEGIN;          //开始拨弹
				}
		}
		
		else if (handle->trigger_state == TRIGGER_BEGIN)        //开始拨弹模式
		{
				if (handle->fire_bullet_number != 0)                //射弹数不为0
        {
             handle->trigger_state = TRIGGERING;             //拨弹中状态
             handle->trigger_motor.set_angle = handle->trigger_motor.angle + (360.0f / TRIGGER_PLATE_NUMBERS);
				}
				
				else
        {
             handle->trigger_state = TRIGGER_END;            //拨弹结束
        }
		}
		
		else if (handle->trigger_state == TRIGGERING)           //拨弹中模式
    {
				if ( fabs(handle->trigger_motor.set_angle - handle->trigger_motor.angle) < 3.0f )        
        {
             handle->fire_bullet_number--;                                                       
             handle->trigger_state = TRIGGER_BEGIN;                                               //拨弹开始
        }
		}
		
	}
	
	else
	{
		handle->trigger_state = TRIGGER_END;		//拨弹结束
    handle->trigger_motor.set_angle = handle->trigger_motor.angle;
	}
	
	handle->trigger_motor.current_set = DoublePID_Calc(&handle->trigger_motor.pid,                  
                                                       handle->trigger_motor.set_angle,
                                                       handle->trigger_motor.angle,
                                                       handle->trigger_motor.speed);
	
}

static void Shoot_FrictionWheelMotorCtrl(ShootCtrlMode_e mode, FrictionWheelMotor_t motor[2])
{
	if (mode == SHOOT_START)
  {
		motor[0].set_speed = 3000;
    motor[1].set_speed = -3000;
	}
	
	else
	{
		motor[0].set_speed = 0;
    motor[1].set_speed = 0;
	}
	
	for (uint8_t i = 0; i < 2; i++)
  {
    motor[i].current_set = pid_calc(&motor[i].pid, motor[i].motor_info->speed_rpm, motor[i].set_speed);
  }
}
