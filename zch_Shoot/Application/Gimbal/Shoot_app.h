#ifndef __SHOOT_APP_H__
#define __SHOOT_APP_H__

#include "main.h"
#include "motor.h"
#include "pid.h"
#include "infantry_console.h"
#include "struct_typedef.h" 

typedef enum
{
    SHOOT_RELAX = 0,          //安全模式
    SHOOT_START,
    SHOOT_STOP,
} ShootCtrlMode_e;

typedef enum
{
    TRIGGER_END = 0,
    TRIGGER_BEGIN,
    TRIGGERING
} TriggerState_e;

typedef enum
{
    MAGAZINE_INIT_STATE = 0,
    MAGAZINE_OFF_STATE,
    MAGAZINE_ON_STATE,
} MagazineState_e;

typedef enum
{
    SHOOT_LEVEL1 = 0,
    SHOOT_LEVEL2,
    SHOOT_LEVEL3
} ShootLevel_e;

typedef struct
{
    MotorInfo_t*    motor_info;
    int32_t         offset_ecd;
    fp32            ecd_ratio;

    Double_PID_t    pid;
    fp32            speed;
    fp32            angle;
    fp32            set_speed;
    fp32            set_angle;
	
	  int16_t         current_set;
		int16_t         GM996R_pwm;

} ShootMotor_t;

typedef struct
{
    MotorInfo_t*    motor_info;

    pid_t           pid;
    fp32            set_speed;

    int16_t         current_set;
} FrictionWheelMotor_t;

typedef struct
{
	Console_t*      console;
	
	ShootCtrlMode_e ctrl_mode;
  ShootMotor_t    magazine_motor;
  ShootMotor_t    trigger_motor;
  FrictionWheelMotor_t  fric_wheel_motor[2];
	
	TriggerState_e  trigger_state;
	
	MagazineState_e  magazine_state;
	
	uint16_t        trigger_last_angle;
  uint16_t        trigger_angle;
	
	uint16_t        fire_bullet_number;
	
}ShootHandle_t;


#define MAGAZINE_MOTOR_POSITIVE_DIR     (-1.0f)
#define MAGAZINE_MOTOR_REDUCTION_RATIO  M2006_REDUCTION_RATIO
#define TRIGGER_MOTOR_POSITIVE_DIR      (-1.0f)
#define TRIGGER_MOTOR_REDUCTION_RATIO   M2006_REDUCTION_RATIO
#define ONE_BULLET_HEAT         (10u)
#define TRIGGER_PLATE_NUMBERS   (8.0f)
#define SHOOT_TASK_PERIOD       3
#define MAGAZINE_MOTOR_INIT_SET         (1000)

void ShootTaskConfig(void);

#endif
