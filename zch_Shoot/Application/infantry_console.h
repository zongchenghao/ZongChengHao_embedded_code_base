#ifndef __INFANTRY_CONSOLE_H__
#define __INFANTRY_CONSOLE_H__

#include "main.h"
#include "remote_control.h"

typedef enum
{
    PREPARE_MODE = 0,       //��ʼ��
    NORMAL_MODE,            //��������ģʽ
    SAFETY_MODE,              //��ȫģʽ��ֹͣ�˶���
} CtrlMode_e;

typedef enum
{
    GIMBAL_RELEASE_CMD = 0,
    GIMBAL_INIT_CMD,
    GIMBAL_GYRO_CMD,
    GIMBAL_RELATIVE_CMD,
    GIMBAL_NORMAL_CMD,
    GIMBAL_VISION_AIM_CMD,
    GIMBAL_ADD_CMD,
} Gimbal_CMD_e;

typedef enum
{
    CHASSIS_RELEASE_CMD = 0,
    CHASSIS_STOP_CMD,               //����ֹͣ
    CHASSIS_FOLLOW_GIMBAL_CMD,      //���̸�����̨
    CHASSIS_SEPARATE_GIMBAL_CMD,    //������̨����
    CHASSIS_SPIN_CMD,               //������ת
//	  CHASSIS_HALF_CMD,               //��ͷ
//    CHASSIS_HALFSPIN_CMD,           //˦β
    CHASSIS_TEST_CMD,               //����ģʽ
} Chassis_CMD_e;

typedef enum
{
    SHOOT_RELEASE_CMD = 0,
    SHOOT_START_CMD,
    SHOOT_STOP_CMD,
} Shoot_CMD_e;

typedef enum
{
    MAGAZINE_INIT_CMD = 0,
    MAGAZINE_OFF_CMD,
    MAGAZINE_ON_CMD,
} Magazine_CMD_e;

typedef enum
{
    STOP_FIRE_CMD = 0,
    ONE_FIRE_CMD,
    RAPID_FIRE_CMD,
} ShootFire_CMD_e;

typedef enum
{
    FLYING_SLOPE_ON_CMD=0,
	  FLYING_SLOPE_OFF_CMD,
}Flying_Slope;

typedef enum
{
    ON = 0,
    OFF,
}flag_e;

typedef struct
{
    RC_Info_t* rc;
    CtrlMode_e ctrl_mode;
		
    Gimbal_CMD_e gimbal_cmd;
    Chassis_CMD_e chassis_cmd;
		Shoot_CMD_e shoot_cmd;
		Magazine_CMD_e magazine_cmd;
		
		
    flag_e shift_flag;
    Flying_Slope flying_slope_cmd;
		uint8_t capbuff;
	
    struct
    {
        float vx;
        float vy;
        float vw;
    } chassis;

    struct
    {
        float pitch_v;
        float yaw_v;
    } gimbal;
		
		struct
		{
			ShootFire_CMD_e fire_cmd;
		}shoot;

		float spin_rate;
		uint8_t keyboard_mode;	
} Console_t;

#define CONSOLE_TASK_PERIOD         20

void ConsoleTaskInit(void);
Console_t* Console_Pointer(void);

#endif
