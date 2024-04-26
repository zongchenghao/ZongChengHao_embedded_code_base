#include "infantry_console.h"
#include "cmsis_os.h"
#include "Gimbal_steer_app.h"
#include "Shoot_app.h"

uint16_t Spin_flag;
Console_t console;
RC_Info_t last_rc;
RC_Switch_t wheel_switch;
extern GimbalHandle_t gimbal_handle;

static void RemoteControlWheelAction(void);
static void RemoteControl_Operation(void);

void Console_Task(void const * argument)
{
//	ConsoleTaskInit();
  for(;;)
  {
		RemoteControlWheelAction();
		switch (console.ctrl_mode)
    {
			case PREPARE_MODE:
     {
			if(gimbal_handle.ctrl_mode != GIMBAL_INIT && gimbal_handle.ctrl_mode != GIMBAL_RELAX)
     {
        console.ctrl_mode = NORMAL_MODE;
			  console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
		 }
		 else
		 {
			  console.gimbal_cmd = GIMBAL_INIT_CMD;
        console.chassis_cmd = CHASSIS_STOP_CMD;
		 }
	   }break;
		 
		 case NORMAL_MODE:
     {
//			if (console.rc->sw1 == REMOTE_SWITCH_VALUE_CENTRAL)
//      {
				RemoteControl_Operation();    
//      }
			
		 }break;
		 
		 case SAFETY_MODE:
     {
			  console.gimbal_cmd  = GIMBAL_RELEASE_CMD;
				console.chassis_cmd  = CHASSIS_RELEASE_CMD;
					
		 }break;
		 
		 default:
        break;	
  }
		
	last_rc = *console.rc;
  osDelay(CONSOLE_TASK_PERIOD);
	
	}
	
}


void ConsoleTaskInit(void)
{
	console.rc = RC_GetDataPointer();
	console.ctrl_mode = NORMAL_MODE;
	console.chassis_cmd = CHASSIS_STOP_CMD;
	console.gimbal_cmd = GIMBAL_INIT_CMD;
	console.shoot_cmd = SHOOT_STOP_CMD;
  console.shoot.fire_cmd = STOP_FIRE_CMD;
	console.shift_flag = OFF;
	console.flying_slope_cmd=FLYING_SLOPE_OFF_CMD;	 
}

Console_t* Console_Pointer(void)
{
    return &console;
}

/*²¦ÂÖ*/
static void RemoteControlWheelAction(void)
{
    static uint8_t wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    if (console.rc->wheel < -440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_UP;
    }
    else if (console.rc->wheel > -220 && console.rc->wheel < 220)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    }
    else if (console.rc->wheel > 440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_DOWN;
    }
    RC_SwitchAction(&wheel_switch, wheel_sw);
}

static void RemoteControl_Operation(void)
{
	
	static uint32_t shoot_time = 0;
  static uint32_t magazine_flag=0; 
	static uint32_t remotecontrol_loose_time=0;
	
	if (console.rc->sw2 == REMOTE_SWITCH_VALUE_CENTRAL)            
    {
				console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;				//ÔÆÌ¨µ×ÅÌ¸úËæ

        console.chassis.vx = -(console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X);
        console.chassis.vy = -(console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y);
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
	
		}
		
		 else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)          //Ð¡ÍÓÂÝ
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_SPIN_CMD;

        console.chassis.vx = console.rc->ch4 / -RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = console.rc->ch3 / -RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
		}
		
		 else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_UP)            //·ÖÀë
    {  
        console.gimbal_cmd  =  GIMBAL_NORMAL_CMD;
		    console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
			
        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * -RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * -RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
		
		if (console.shoot_cmd == SHOOT_STOP_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_START_CMD;
        }
				
				if(wheel_switch.switch_value_raw== REMOTE_SWITCH_VALUE_DOWN&&magazine_flag==1&&remotecontrol_loose_time>50)
				{
						console.magazine_cmd = MAGAZINE_OFF_CMD;
						magazine_flag=0;
						remotecontrol_loose_time=0;
				}
				
				if(wheel_switch.switch_value_raw== REMOTE_SWITCH_VALUE_DOWN&&magazine_flag==0&&remotecontrol_loose_time>50)
				{
						console.magazine_cmd = MAGAZINE_ON_CMD;
						magazine_flag=1;
						remotecontrol_loose_time=0;
				}
				
				remotecontrol_loose_time++;
		}
		
		else if (console.shoot_cmd == SHOOT_START_CMD)
    {
			
			 if(console.magazine_cmd== MAGAZINE_ON_CMD)
			 {
						console.magazine_cmd = MAGAZINE_OFF_CMD;
						magazine_flag=0;
			 }
			 
			 if(last_rc.sw1 == REMOTE_SWITCH_VALUE_DOWN)
       {
						console.shoot_cmd = SHOOT_STOP_CMD;
       }
			
			 if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
       {
						console.shoot_cmd = SHOOT_STOP_CMD;
       }
			 
			 else if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO2)
       {
            console.shoot.fire_cmd = ONE_FIRE_CMD;
       }
			 
			 else if (wheel_switch.switch_value_raw == REMOTE_SWITCH_VALUE_DOWN)
       {
            shoot_time++;
            if(shoot_time > 50)
                console.shoot.fire_cmd = ONE_FIRE_CMD ;
            else
                console.shoot.fire_cmd = STOP_FIRE_CMD;
       }
			 
			 else
			 {
					  console.shoot.fire_cmd = STOP_FIRE_CMD;
            shoot_time = 0;
			 }
		}
}
