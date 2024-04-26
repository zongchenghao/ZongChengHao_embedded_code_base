#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "main.h"
#include "usart.h"
#include "motor.h"

typedef struct
{
    /* 摇杆 */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    /* 拨杆 */
    uint8_t sw1;
    uint8_t sw2;
    /* 鼠标 */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t l;
        uint8_t r;
    } mouse;
    /* 键盘 */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        } bit;
    } kb;
    /* 拨轮 */
    int16_t wheel;
} RC_Info_t;

typedef struct
{
    uint8_t switch_value_raw;       // 当前开关值
    uint8_t last_switch_value_raw;  // 上次开关值
    uint8_t switch_state;           // 状态
} RC_Switch_t;


#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3 
#define RC_DEADBAND         (5)         // 遥控器死区，因为遥控器的拨杆在中位时不一定为0
#define RC_STICK_OFFSET     (1024u)     // 拨杆中间值
#define RC_VALUE_MIN        (364u)
#define RC_VALUE_MAX        (1684u)
#define RC_RESOLUTION       (660.0f)    // 遥控取值范围 做转化用
#define MAX_VX_SPEED        (6900.0f)

#define REMOTE_SWITCH_VALUE_UP          0x01u
#define REMOTE_SWITCH_VALUE_DOWN        0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL     0x03u
#define REMOTE_SWITCH_CHANGE_1TO3      (uint8_t)((REMOTE_SWITCH_VALUE_UP << 2) | REMOTE_SWITCH_VALUE_CENTRAL)
#define REMOTE_SWITCH_CHANGE_2TO3      (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 2) | REMOTE_SWITCH_VALUE_CENTRAL)
#define REMOTE_SWITCH_CHANGE_3TO1      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_UP)
#define REMOTE_SWITCH_CHANGE_3TO2      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_DOWN)

/*-----------------↓ 遥控 ↓-----------------*/
#define RC_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     
#define RC_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     
#define RC_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED     
#define RC_GIMBAL_MOVE_RATIO_PIT    0.0005f       
#define RC_GIMBAL_MOVE_RATIO_YAW    0.002f       
/*---------------↓ 鼠标键盘 ↓---------------*/
#define KB_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X轴方向最大速度
#define KB_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y轴方向最大速度
#define KB_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED      
#define KB_GIMBAL_MOVE_RATIO_PIT    0.005f      
#define KB_GIMBAL_MOVE_RATIO_YAW    0.01f       
#define VS_GIMBAL_MOVE_RATIO_YAW    0.15f         
#define VS_GIMBAL_MOVE_RATIO_PIT    0.15f         

#define CHASSIS_ACCEL_TIME      1500  //ms           
#define ROTATE_ACCEL_TIME       3000  //ms           
#define CHASSIS_SHIFT_ACCEL_TIME      2161  //ms           

void RC_DataParser(RC_Info_t *rc, uint8_t *buf);
void RC_SwitchAction(RC_Switch_t *sw, uint8_t value);
RC_Info_t* RC_GetDataPointer(void);
void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);

#endif
