#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "AHRS.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    float gyro[3];       /* 角速度 */
    float accel[3];      /* 加速度 */
    float mag[3];        /* 磁力计 */
    float temp;          /* 温度 */
    float temp_tar;      /* 目标温度 */
    float quat[4];       /* 四元数 */

    /* 欧拉角 单位 rad */
    struct
    {
        float pitch;
        float roll;
        float yaw;
    } euler;

    /* 姿态角 */
    struct
    {
        float pitch;
        float roll;
        float yaw;
    } attitude;
} IMU_Data_t;

#define IMU_TASK_PERIOD             5

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void IMU_Init(void);
void IMU_Update(const fp32 period_time);
IMU_Data_t* IMU_GetDataPointer(void);
void IMU_CalibrateGyro(fp32 cali_offset[3]);
void IMU_SetGyroOffset(fp32 cali_offset[3]);

#endif  // IMU_DRIVER_H

