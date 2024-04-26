#include "Text_task.h"

extern motor_info_t chassis_motor[4];
extern rc_info_t rc;

void Text_Task(void const *pvParameters)
{
	for(;;)
	{
		motor_pwm_set(500);
		osDelay(500);
		motor_pwm_set(1700);
		osDelay(500);
	}
}
