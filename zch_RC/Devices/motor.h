#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;

    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;

    int32_t round_cnt;
    int32_t total_ecd;
    int32_t total_angle;

    int32_t ecd_raw_rate;

    uint32_t msg_cnt;
    uint16_t offset_ecd;
} motor_info_t;

void motor_data_parse(motor_info_t *ptr, uint8_t data[]);

int16_t Motor_RelativePosition(int16_t ecd, int16_t center_offset);

int16_t Motor_Relativeangle(int16_t angle, int16_t angle_offset);

#endif
