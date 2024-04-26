#include "imu_task.h"
#include "cmsis_os.h"
#include "imu_driver.h"
#include "pid.h"
#include "tim.h"

#define IMU_TEMPERATURE    40.0f

#define TEMPERATURE_PID_KP 1.0f
#define TEMPERATURE_PID_KI 0.2f
#define TEMPERATURE_PID_KD 0.0f

#define TEMPERATURE_PID_MAX_OUT  4500.0f
#define TEMPERATURE_PID_MAX_IOUT 4400.0f

#define TEMP_PWM_MAX 5000

IMU_Data_t* pimu;
pid_t imu_temp_pid; // IMU温度控制PID
static uint8_t first_temperate = 0;

void IMU_Init(void);

static void IMU_TempControl(fp32 temp);

void IMU_Task(void const * argument)
{
	IMU_TaskInit();
	uint32_t period = osKernelSysTick();
//	HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
  for(;;)
  {
		IMU_Update((fp32)IMU_TASK_PERIOD / 1000.0f);
		IMU_TempControl(40);
		osDelayUntil(&period, IMU_TASK_PERIOD);
  }
  /* USER CODE END IMU_Task */
}

void IMU_TaskInit(void)
{
    IMU_Init();
		pimu = IMU_GetDataPointer();
		pid_init(&imu_temp_pid,
             POSITION_PID,
             TEMPERATURE_PID_MAX_OUT,
             TEMPERATURE_PID_MAX_IOUT,
             TEMPERATURE_PID_KP,
             TEMPERATURE_PID_KI,
             TEMPERATURE_PID_KD);
}

/**
  * @brief 控制BMI088的温度
  */
static void IMU_TempControl(fp32 temp)
{
    uint16_t pwm;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        pid_calc(&imu_temp_pid, temp, IMU_TEMPERATURE);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        pwm = (uint16_t)imu_temp_pid.out;
				__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
    }
    else
    {
        //in beginning, max power
        if (temp > IMU_TEMPERATURE)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                first_temperate = 1;
                imu_temp_pid.iout = TEMP_PWM_MAX / 2.0f;
            }
        }
				__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, TEMP_PWM_MAX-1);
    }
}
