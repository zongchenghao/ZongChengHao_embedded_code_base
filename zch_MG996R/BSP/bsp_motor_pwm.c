#include "bsp_motor_pwm.h"


void motor_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
}



