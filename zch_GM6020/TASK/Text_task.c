#include "Text_task.h"

extern motor_info_t GM6020_Moter[4];
extern rc_info_t rc;

void Text_Task(void const *pvParameters)
{
	for(;;)
	{
		CAN_GM6020_Send(&hcan1,5000, 5000, 0, 0);//��C���CAN1�ڣ����IDΪID1
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
						  if(RxMeg.StdId == 0x205)//���ID����ΪID1
						 {
                motor_data_parse(&GM6020_Moter[0], rx_data);
						 }
				}
			      
				if(hcan->Instance == CAN2)
				{
						HAL_RetVal=HAL_CAN_GetRxMessage(&hcan2,  CAN_RX_FIFO0, &RxMeg,  rx_data);
				}
		} 
}
