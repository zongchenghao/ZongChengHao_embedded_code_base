#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

typedef struct
{
    uint16_t ecd;	//电机编码器的读数
    uint16_t last_ecd; //上一次读取电机编码器时的值

    int16_t speed_rpm; //电机速度，单位为 RPM
    int16_t given_current; //电机给定电流
    uint8_t temperature;           //电机温度

    int32_t round_cnt; //电机旋转圈数
    int32_t total_ecd; //电机编码器总的累计读数
    int32_t total_angle; //电机旋转角度的总和，单位为度

    int32_t ecd_raw_rate; //电机编码器读数的原始变化率

    uint32_t msg_cnt; //消息计数器，计算电机消息的数量
    uint16_t offset_ecd; //电机编码器的初始值（偏移量），用于编码器读数的校准
} MotorInfo_t;

typedef struct
{
    uint16_t        positive_offset_ecd;
    uint16_t        reverse_offset_ecd;
    uint16_t        positive45_offset_ecd;
	uint16_t      chassis_offset_ecd;
} Offset_ecd;

#define MOTOR_1_FEEDBACK_ID         0x201
#define MOTOR_2_FEEDBACK_ID         0x202
#define MOTOR_3_FEEDBACK_ID         0x203
#define MOTOR_4_FEEDBACK_ID         0x204
#define MOTOR_5_FEEDBACK_ID         0x205
#define MOTOR_6_FEEDBACK_ID         0x206
#define MOTOR_7_FEEDBACK_ID         0x207
#define MOTOR_8_FEEDBACK_ID         0x208
#define GIMBAL_MOTOR1_FEEDBACK_ID   0x209
#define GIMBAL_MOTOR2_FEEDBACK_ID   0x20A
#define GIMBAL_MOTOR3_FEEDBACK_ID   0x20B

#define MOTOR_ENCODER_RANGE         (8192)
#define MOTOR_ENCODER_RANGE_HALF    (4096)
#define ENCODER_ANGLE_RATIO         (8192.0f / 360.0f)

/*底盘电机*/
#define CHASSIS_MOTOR_LF_MESSAGE_ID     MOTOR_1_FEEDBACK_ID     //底盘左前电机   //LF
#define CHASSIS_MOTOR_RF_MESSAGE_ID     MOTOR_2_FEEDBACK_ID     //底盘右前电机
#define CHASSIS_MOTOR_LB_MESSAGE_ID     MOTOR_3_FEEDBACK_ID     //底盘左后电机
#define CHASSIS_MOTOR_RB_MESSAGE_ID     MOTOR_4_FEEDBACK_ID     //底盘右后电机   //RB

#define CHASSIS_STEER_MOTOR_LF_MESSAGE_ID     MOTOR_5_FEEDBACK_ID     //底盘左前云台电机
#define CHASSIS_STEER_MOTOR_RF_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     //底盘右前云台电机
#define CHASSIS_STEER_MOTOR_LB_MESSAGE_ID     MOTOR_7_FEEDBACK_ID     //底盘左后云台电机
#define CHASSIS_STEER_MOTOR_RB_MESSAGE_ID     MOTOR_8_FEEDBACK_ID     //底盘右后云台电机

/*云台电机*/
#define GIMBAL_PITCH_MOTOR       MOTOR_5_FEEDBACK_ID
#define GIMBAL_YAW_MOTOR         MOTOR_6_FEEDBACK_ID

/*发射机构*/
#define FRICTION_WHEEL_1_MESSAGE_ID     MOTOR_1_FEEDBACK_ID
#define FRICTION_WHEEL_2_MESSAGE_ID     MOTOR_2_FEEDBACK_ID
#define TRIGGER_MOTOR_MESSAGE_ID        MOTOR_3_FEEDBACK_ID
#define MAGAZINE_MOTOR_MESSAGE_ID       MOTOR_4_FEEDBACK_ID

/*标识符*/
#define MOTOR_1TO4_CONTROL_STD_ID   0x200
#define MOTOR_5TO8_CONTROL_STD_ID   0x1FF

#define M3508_MOTOR_MAX_CURRENT     (16000.0f)
#define GM6020_MOTOR_MAX_CURRENT    (30000.0f)
#define M2006_MOTOR_MAX_CURRENT     (10000.0f)

#define MAX_CHASSIS_VX_SPEED        (6900.0f)
#define MAX_CHASSIS_VY_SPEED        (6900.0f)
#define MAX_CHASSIS_VW_SPEED        (500.0f)

#define M3508_REDUCTION_RATIO       (1.0f/19.0f)        //3508减速比
#define M2006_REDUCTION_RATIO       (1.0f/36.0f)        //2006减速比

int16_t Motor_RelativePosition(int16_t ecd, int16_t center_offset);
int16_t Motor_Relativeangle(int16_t angle, int16_t angle_offset);
void Motor_SendMessage(CAN_HandleTypeDef* hcan, uint32_t std_id,int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4);
void Motor_EncoderData(MotorInfo_t* ptr, uint8_t data[]);

MotorInfo_t* ChassisMotor_Pointer(uint8_t i);
MotorInfo_t* Chassis_steer_Motor_Pointer(uint8_t i);
MotorInfo_t* Gimbal_pitch_Motor_Pointer(void);
MotorInfo_t* Gimbal_yaw_Motor_Pointer(void);
MotorInfo_t* FrictionWheelMotor_1_Pointer(void);
MotorInfo_t* FrictionWheelMotor_2_Pointer(void);
MotorInfo_t* TriggerMotor_Pointer(void);
//MotorInfo_t* MagazineMotor_Pointer(void);


#endif
