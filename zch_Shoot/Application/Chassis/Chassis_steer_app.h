#ifndef __CHASSIS_STEER_APP_H__
#define __CHASSIS_STEER_APP_H__

#include "main.h"
#include "motor.h"
#include "pid.h"
#include "imu_driver.h"
#include "infantry_console.h"

typedef struct
{
	 pid_t           outer_pid;
   pid_t           inter_pid;
   fp32            angle_ref;
   fp32            angle_fdb;
   fp32            speed_ref;
   fp32            speed_fdb;
}Chassis_steer_pid_t;

typedef enum       //模式
{
    CHASSIS_RELAX = 0,          
    CHASSIS_STOP,               
    CHASSIS_FOLLOW_GIMBAL,      
    CHASSIS_SEPARATE_GIMBAL,    
    CHASSIS_SPIN,               
	  CHASSIS_TEST,
} ChassisCtrlMode_e;

typedef struct      
{
    fp32 wheel_perimeter; /* the perimeter(mm) of wheel *///轮周长
    fp32 wheeltrack;      /* wheel track distance(mm) */  //轮距
    fp32 wheelbase;       /* wheelbase distance(mm) */    //轴距
    fp32 rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */  //相对于底盘中心的x轴旋转偏移（mm）
    fp32 rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */  //相对于底盘中心的y轴旋转偏移（mm）
	
	  fp32 Radius;    //中心半径
} MechanicalStructure_t;

typedef struct Steer_Type
{
  float  vect_angle;     
  float  vect_angleAbs;  //!
  float  str_angle;      //!<Steer angle
  int8_t speed_direction;//!<Speed direction of steer, only contains two state
  int8_t angle_direction;              //
}Steer_Type;

typedef struct         //电机数据
{
    MotorInfo_t*    motor_info;
    pid_t           pid;
    fp32            given_speed;
    int16_t         current_set;
} ChassisMotor_t;

typedef struct
{
    fp32            relative_angle; /* unit: degree */
    fp32            gyro_angle;
    fp32            palstance;      /* uint: degree/s */
} Sensor_t;

typedef struct
{
	MotorInfo_t*            motor_info;
	Chassis_steer_pid_t     pid;
	fp32                    given_value;
  int16_t                 current_set;
	Offset_ecd              offset_ecd;
	fp32                    ecd_ratio;//转向系数
	fp32                    max_relative_angle;
  fp32                    min_relative_angle;
	Sensor_t                sensor;
	
}Chassis_steer_Motor_t;

typedef struct
{
	Console_t*              console;
	IMU_Data_t*             imu;
	
	MechanicalStructure_t   structure;
	ChassisMotor_t          chassis_motor[4];
	Chassis_steer_Motor_t   chassis_steer_motor[4];
	
	ChassisCtrlMode_e       ctrl_mode;
	
	fp32                    gimbal_yaw_ecd_angle;
	
	pid_t                   chassis_follow_pid;
	
	fp32                    vx;                      
  fp32                    vy;                      
  fp32                    vw;                      
  fp32                    wheel_rpm[4];
	
	fp32                    steeringAngleTarget[4];               
	fp32                    lastSteeringAngletarget[4];
	fp32                    steeringAngle[4];                     
	fp32                    last_steeringAngle[4];
	int32_t                 motor_circle[4];
	int32_t                 motor_target_count[4];
	Steer_Type              steer_set[4];
	uint16_t                turnFlag[4];       //舵轮角度解算
	
	fp32                    chassis_yaw;   
  fp32                    chassis_pitch; 
  fp32                    chassis_roll;
	
	
}ChassisHandle_t;

#define RADIUS                      (248.195f)   //中心半径(mm)
#define WHEEL_RADIUS                (60)    //轮子半径(mm)
#define WHEEL_PERIMETER             (376.99)   //轮子周长(mm)
#define WHEELTRACK                  (351)   
#define WHEELBASE                   (351)
#define STEER_MOTO_POSITIVE_DIR     (1.0f)
#define STEER_REDUCTION_RATIO       (1.0f)  

void Chassis_steerAppConfig(void);
void calculateRoundCnt(ChassisHandle_t* chassis_handle);
void calculateTargetRoundCnt(ChassisHandle_t* chassis_handle);

#endif
