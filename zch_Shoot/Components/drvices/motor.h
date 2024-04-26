#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

typedef struct
{
    uint16_t ecd;	//����������Ķ���
    uint16_t last_ecd; //��һ�ζ�ȡ���������ʱ��ֵ

    int16_t speed_rpm; //����ٶȣ���λΪ RPM
    int16_t given_current; //�����������
    uint8_t temperature;           //����¶�

    int32_t round_cnt; //�����תȦ��
    int32_t total_ecd; //����������ܵ��ۼƶ���
    int32_t total_angle; //�����ת�Ƕȵ��ܺͣ���λΪ��

    int32_t ecd_raw_rate; //���������������ԭʼ�仯��

    uint32_t msg_cnt; //��Ϣ����������������Ϣ������
    uint16_t offset_ecd; //����������ĳ�ʼֵ��ƫ�����������ڱ�����������У׼
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

/*���̵��*/
#define CHASSIS_MOTOR_LF_MESSAGE_ID     MOTOR_1_FEEDBACK_ID     //������ǰ���   //LF
#define CHASSIS_MOTOR_RF_MESSAGE_ID     MOTOR_2_FEEDBACK_ID     //������ǰ���
#define CHASSIS_MOTOR_LB_MESSAGE_ID     MOTOR_3_FEEDBACK_ID     //���������
#define CHASSIS_MOTOR_RB_MESSAGE_ID     MOTOR_4_FEEDBACK_ID     //�����Һ���   //RB

#define CHASSIS_STEER_MOTOR_LF_MESSAGE_ID     MOTOR_5_FEEDBACK_ID     //������ǰ��̨���
#define CHASSIS_STEER_MOTOR_RF_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     //������ǰ��̨���
#define CHASSIS_STEER_MOTOR_LB_MESSAGE_ID     MOTOR_7_FEEDBACK_ID     //���������̨���
#define CHASSIS_STEER_MOTOR_RB_MESSAGE_ID     MOTOR_8_FEEDBACK_ID     //�����Һ���̨���

/*��̨���*/
#define GIMBAL_PITCH_MOTOR       MOTOR_5_FEEDBACK_ID
#define GIMBAL_YAW_MOTOR         MOTOR_6_FEEDBACK_ID

/*�������*/
#define FRICTION_WHEEL_1_MESSAGE_ID     MOTOR_1_FEEDBACK_ID
#define FRICTION_WHEEL_2_MESSAGE_ID     MOTOR_2_FEEDBACK_ID
#define TRIGGER_MOTOR_MESSAGE_ID        MOTOR_3_FEEDBACK_ID
#define MAGAZINE_MOTOR_MESSAGE_ID       MOTOR_4_FEEDBACK_ID

/*��ʶ��*/
#define MOTOR_1TO4_CONTROL_STD_ID   0x200
#define MOTOR_5TO8_CONTROL_STD_ID   0x1FF

#define M3508_MOTOR_MAX_CURRENT     (16000.0f)
#define GM6020_MOTOR_MAX_CURRENT    (30000.0f)
#define M2006_MOTOR_MAX_CURRENT     (10000.0f)

#define MAX_CHASSIS_VX_SPEED        (6900.0f)
#define MAX_CHASSIS_VY_SPEED        (6900.0f)
#define MAX_CHASSIS_VW_SPEED        (500.0f)

#define M3508_REDUCTION_RATIO       (1.0f/19.0f)        //3508���ٱ�
#define M2006_REDUCTION_RATIO       (1.0f/36.0f)        //2006���ٱ�

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
