#include "motor.h"

motor_info_t GM6020_Moter[4];

static void motor_encoder_data(motor_info_t* ptr, uint8_t data[])
{
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);

    if (ptr->ecd - ptr->last_ecd > 4096)
    {
        ptr->round_cnt--;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
    }
    else if (ptr->ecd - ptr->last_ecd < -4096)
    {
        ptr->round_cnt++;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
    }
    else
    {
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
    }

		ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd;
    ptr->total_angle = ptr->total_ecd / (8192.0f / 360.0f);

    ptr->speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    ptr->given_current = (int16_t)(data[4] << 8 | data[5]);
    ptr->temperature = data[6];
}

static void motor_encoder_offset(motor_info_t *ptr, uint8_t data[])
{
    ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_ecd = ptr->ecd;
}

void motor_data_parse(motor_info_t *ptr, uint8_t data[])
{
    if (ptr == NULL)
        return;
    ptr->msg_cnt++;

    if (ptr->msg_cnt < 50)
    {
        motor_encoder_offset(ptr, data);
        return;
    }

    motor_encoder_data(ptr, data);
}

int16_t Motor_RelativePosition(int16_t ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= 4096)
    {
        if (ecd > center_offset - 4096)
            tmp = ecd - center_offset;
        else
            tmp = ecd + 8192 - center_offset;
    }
    else
    {
        if (ecd > center_offset + 4096)
            tmp = ecd - 8192 - center_offset;
        else
            tmp = ecd - center_offset;
    }
    return tmp;
}

int16_t Motor_Relativeangle(int16_t angle, int16_t angle_offset)
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


