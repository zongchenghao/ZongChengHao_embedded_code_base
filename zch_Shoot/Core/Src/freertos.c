/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId test_TaskHandle;
osThreadId imu_taskHandle;
osThreadId console_taskHandle;
osThreadId chassis_taskHandle;
osThreadId gimbal_steer_taHandle;
osThreadId shoot_taskHandle;
osTimerId test_TimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void TestTask(void const * argument);
void IMU_Task(void const * argument);
void Console_Task(void const * argument);
void Chassis_Task(void const * argument);
void Gimbal_Steer_Task(void const * argument);
void ShootTask(void const * argument);
void TestTimerCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of test_Timer */
  osTimerDef(test_Timer, TestTimerCallback);
  test_TimerHandle = osTimerCreate(osTimer(test_Timer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of test_Task */
  osThreadDef(test_Task, TestTask, osPriorityNormal, 0, 256);
  test_TaskHandle = osThreadCreate(osThread(test_Task), NULL);

  /* definition and creation of imu_task */
  osThreadDef(imu_task, IMU_Task, osPriorityNormal, 0, 256);
  imu_taskHandle = osThreadCreate(osThread(imu_task), NULL);

  /* definition and creation of console_task */
  osThreadDef(console_task, Console_Task, osPriorityNormal, 0, 256);
  console_taskHandle = osThreadCreate(osThread(console_task), NULL);

  /* definition and creation of chassis_task */
  osThreadDef(chassis_task, Chassis_Task, osPriorityNormal, 0, 256);
  chassis_taskHandle = osThreadCreate(osThread(chassis_task), NULL);

  /* definition and creation of gimbal_steer_ta */
  osThreadDef(gimbal_steer_ta, Gimbal_Steer_Task, osPriorityNormal, 0, 256);
  gimbal_steer_taHandle = osThreadCreate(osThread(gimbal_steer_ta), NULL);

  /* definition and creation of shoot_task */
  osThreadDef(shoot_task, ShootTask, osPriorityNormal, 0, 256);
  shoot_taskHandle = osThreadCreate(osThread(shoot_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TestTask */
/**
* @brief Function implementing the test_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TestTask */
__weak void TestTask(void const * argument)
{
  /* USER CODE BEGIN TestTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TestTask */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the imu_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
__weak void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_Console_Task */
/**
* @brief Function implementing the console_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Console_Task */
__weak void Console_Task(void const * argument)
{
  /* USER CODE BEGIN Console_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Console_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the chassis_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Gimbal_Steer_Task */
/**
* @brief Function implementing the gimbal_steer_ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Steer_Task */
__weak void Gimbal_Steer_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Steer_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Steer_Task */
}

/* USER CODE BEGIN Header_ShootTask */
/**
* @brief Function implementing the shoot_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ShootTask */
__weak void ShootTask(void const * argument)
{
  /* USER CODE BEGIN ShootTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ShootTask */
}

/* TestTimerCallback function */
__weak void TestTimerCallback(void const * argument)
{
  /* USER CODE BEGIN TestTimerCallback */

  /* USER CODE END TestTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
