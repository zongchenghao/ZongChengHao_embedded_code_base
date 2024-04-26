#include "motor.h"
#include "can.h"

MotorInfo_t chassis_motor[4]; //底盘驱动电机
MotorInfo_t chassis_steer_motor[4]; //底盘转向电机
MotorInfo_t gimbal_pitch_motor;
MotorInfo_t gimbal_yaw_motor;
MotorInfo_t friction_wheel_motor[2];
MotorInfo_t trigger_motor;
//MotorInfo_t magazine_motor;

void Motor_EncoderData(MotorInfo_t* ptr, uint8_t data[])//将编码器、速度、给定电流和温度等其他有关电机状态的信息更新到 MotorInfo_t 结构体中
{
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);
	
		 if (ptr->ecd - ptr->last_ecd > MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt--;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - MOTOR_ENCODER_RANGE;
    }
    else if (ptr->ecd - ptr->last_ecd < -MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt++;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + MOTOR_ENCODER_RANGE;
    }
    else
    {
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
    }

//    ptr->total_ecd = ptr->round_cnt * MOTOR_ENCODER_RANGE + ptr->ecd - ptr->offset_ecd;
		ptr->total_ecd = ptr->round_cnt * MOTOR_ENCODER_RANGE + ptr->ecd;
    /* total angle, unit is degree */
    ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
		
    ptr->speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    ptr->given_current = (int16_t)(data[4] << 8 | data[5]);
    ptr->temperature = data[6];
}

int16_t Motor_RelativePosition(int16_t ecd, int16_t center_offset)//设置编码器中心位置，算出每次要转到该位置的最优解（需要逆时针或顺时针转几度）
{
    int16_t tmp = 0;
    if (center_offset >= MOTOR_ENCODER_RANGE_HALF)
    {
        if (ecd > center_offset - MOTOR_ENCODER_RANGE_HALF)
            tmp = ecd - center_offset;
        else
            tmp = ecd + MOTOR_ENCODER_RANGE - center_offset;
    }
    else
    {
        if (ecd > center_offset + MOTOR_ENCODER_RANGE_HALF)
            tmp = ecd - MOTOR_ENCODER_RANGE - center_offset;
        else
            tmp = ecd - center_offset;
    }
    return tmp;
}

int16_t Motor_Relativeangle(int16_t angle, int16_t angle_offset)//设置电机中心的角度值，算出每次要转到该角度的最优解（需要逆时针或顺时针转几度）
{
    int16_t tmp = 0;
    if (angle_offset >= 180)
    {
        if (angle > angle_offset - 180)
            tmp = angle - angle_offset;
        else
            tmp = angle + 360 - angle_offset;
    }
    else
    {
        if (angle > angle_offset + 180)
            tmp = angle - 360 - angle_offset;
        else
            tmp = angle - angle_offset;
    }
    return tmp;
}

void Motor_SendMessage(CAN_HandleTypeDef* hcan, uint32_t std_id,int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4) //CAN发送函数
{
	
	CAN_TxHeaderTypeDef tx_header;
  uint8_t TxData[8] = {0};
	
  tx_header.StdId = std_id;//实例
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  TxData[0] = (uint8_t)(cur1 >> 8);
  TxData[1] = (uint8_t)cur1;
  TxData[2] = (uint8_t)(cur2 >> 8);
  TxData[3] = (uint8_t)cur2;
  TxData[4] = (uint8_t)(cur3 >> 8);
  TxData[5] = (uint8_t)cur3;
  TxData[6] = (uint8_t)(cur4 >> 8);
  TxData[7] = (uint8_t)cur4;
	
	HAL_CAN_AddTxMessage(hcan, &tx_header, TxData,(uint32_t*)CAN_TX_MAILBOX0);
}

MotorInfo_t* ChassisMotor_Pointer(uint8_t i)
{
    return &chassis_motor[i];
}

MotorInfo_t* Chassis_steer_Motor_Pointer(uint8_t i)
{
    return &chassis_steer_motor[i];
}

MotorInfo_t* Gimbal_pitch_Motor_Pointer(void)
{
    return &gimbal_pitch_motor;
}

MotorInfo_t* Gimbal_yaw_Motor_Pointer(void)
{
    return &gimbal_yaw_motor;
}

MotorInfo_t* FrictionWheelMotor_1_Pointer(void)
{
    return &friction_wheel_motor[0];
}

MotorInfo_t* FrictionWheelMotor_2_Pointer(void)
{
    return &friction_wheel_motor[1];
}

MotorInfo_t* TriggerMotor_Pointer(void)
{
    return &trigger_motor;
}

//MotorInfo_t* MagazineMotor_Pointer(void)
//{
//    return &magazine_motor;
//}
