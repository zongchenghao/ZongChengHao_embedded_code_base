#ifndef __GIMBAL_STEER_APP_H__
#define __GIMBAL_STEER_APP_H__

#include "main.h"
#include "struct_typedef.h"
#include "motor.h"
#include "infantry_console.h"
#include "pid.h"
#include "imu_driver.h"

typedef enum
{
    GIMBAL_RELAX = 0,          //安全模式
    GIMBAL_INIT,
//    GIMBAL_GYRO,
//    GIMBAL_RELATIVE,
    GIMBAL_NORMAL,
//    GIMBAL_VISION_AIM,
//    GIMBAL_ADD,
} GimbalCtrlMode_e;

typedef enum
{
    RAW_VALUE_MODE = 0,
    GYRO_MODE,
    ENCONDE_MODE,
} GimbalMotorMode_e;

typedef struct
{
    fp32            relative_angle; /* unit: degree */
    fp32            gyro_angle;
    fp32            palstance;      /* uint: degree/s */
} GimbalSensor_t;

typedef struct
{
    pid_t           outer_pid;
    pid_t           inter_pid;
    fp32            angle_ref;
    fp32            angle_fdb;
    fp32            speed_ref;
    fp32            speed_fdb;
} Gimbal_PID_t;

typedef struct
{
    MotorInfo_t*      motor_info;
    Offset_ecd        offset_ecd;
    fp32              ecd_ratio; //编码器的分辨率
    fp32              max_relative_angle;
    fp32              min_relative_angle;

    GimbalMotorMode_e mode;
    GimbalMotorMode_e last_mode;
	
    fp32              given_value;
	  fp32              last_given_value;
    GimbalSensor_t    sensor;
    Gimbal_PID_t      pid;

    int16_t           current_set;
} GimbalMotor_t;

typedef struct
{
    Console_t*           console;
    IMU_Data_t*          imu;              
    
    Gimbal_CMD_e         last_cmd;
    GimbalCtrlMode_e     ctrl_mode;       
    GimbalCtrlMode_e     last_ctrl_mode;

    GimbalMotor_t        yaw_motor;
    GimbalMotor_t        pitch_motor;
} GimbalHandle_t;

#define PITCH_REDUCTION_RATIO       (1.0f)  
#define YAW_REDUCTION_RATIO         (1.0f)  
#define PITCH_MOTO_POSITIVE_DIR     (1.0f)  
#define YAW_MOTO_POSITIVE_DIR       (1.0f)  

void GimbalAppConfig(void);

#endif
