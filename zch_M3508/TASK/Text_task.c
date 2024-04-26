#include "Text_task.h"

extern motor_info_t chassis_motor[4];
extern rc_info_t rc;
uint8_t text;

void Text_Task(void const *pvParameters)
{
	for(;;)
	{
		CAN_M3508_Send(&hcan1,1000, 1000, 0, 0);//接C板的CAN1口，电机ID为ID1
		osDelay(10);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    HAL_StatusTypeDef HAL_RetVal;	
    CAN_RxHeaderTypeDef RxMeg;
    uint8_t	rx_data[8];
    uint8_t id;	

    if(hcan->Instance == CAN1)
    {
        HAL_RetVal=HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMeg,  rx_data);
        if ( HAL_OK==HAL_RetVal)
        {

             if(RxMeg.StdId == 0x201)//电机ID设置为ID1
						 {
                motor_data_parse(&chassis_motor[0], rx_data);
						 }
				}
			      
				if(hcan->Instance == CAN2)
				{
						HAL_RetVal=HAL_CAN_GetRxMessage(&hcan2,  CAN_RX_FIFO0, &RxMeg,  rx_data);
				}
		} 
}
