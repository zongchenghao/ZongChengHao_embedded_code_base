#include "bsp_can.h"

void CAN_FilterInit(CAN_HandleTypeDef* hcan)
{
    CAN_FilterTypeDef CAN_FilterStructure;

    if(hcan->Instance ==CAN1)
        CAN_FilterStructure.FilterBank = 0,CAN_FilterStructure.SlaveStartFilterBank  = 0;
    else if(hcan->Instance ==CAN2)
        CAN_FilterStructure.FilterBank = 14,CAN_FilterStructure.SlaveStartFilterBank  = 14;
    CAN_FilterStructure.FilterActivation = ENABLE;
    CAN_FilterStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterStructure.FilterIdHigh = 0x0000;
    CAN_FilterStructure.FilterIdLow = 0x0000;
    CAN_FilterStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterStructure.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(hcan, &CAN_FilterStructure); 
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint8_t CAN_SendMsg(CAN_HandleTypeDef* hcan,uint8_t ide,uint32_t id,uint8_t len,uint8_t *data)
{
    uint32_t   TxMailbox;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    HAL_StatusTypeDef   HAL_RetVal;
    uint16_t i=0;
    if(ide == 0) {
        CAN_TxHeader.IDE = CAN_ID_STD;	
        CAN_TxHeader.StdId = id;
    }
    else {
        CAN_TxHeader.IDE = CAN_ID_EXT;	
        CAN_TxHeader.ExtId = id;
    }
    CAN_TxHeader.DLC = len;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;
    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        i++;
        if(i>0xfffe)
            return 1;
    }
//    HAL_Delay(1);
    HAL_RetVal = HAL_CAN_AddTxMessage(hcan,&CAN_TxHeader,data,&TxMailbox);
    if(HAL_RetVal != HAL_OK) {
        return 1;
    }
	
    return 0;
}

void CAN_M3508_Send(CAN_HandleTypeDef* hcan,int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
	uint8_t tx_data[8];
  tx_data[0] = data1 >> 8;
  tx_data[1] = data1;
  tx_data[2] = data2 >> 8;
  tx_data[3] = data2;
  tx_data[4] = data3 >> 8;
  tx_data[5] = data3;
  tx_data[6] = data4 >> 8;
  tx_data[7] = data4;
  CAN_SendMsg(hcan,0,0x200,8,tx_data);
}

void CAN_GM6020_Send(CAN_HandleTypeDef* hcan,int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
	uint8_t tx_data[8];
  tx_data[0] = data1 >> 8;
  tx_data[1] = data1;
  tx_data[2] = data2 >> 8;
  tx_data[3] = data2;
  tx_data[4] = data3 >> 8;
  tx_data[5] = data3;
  tx_data[6] = data4 >> 8;
  tx_data[7] = data4;
  CAN_SendMsg(hcan,0,0x1FF,8,tx_data);
}

void CAN_M2006_Send(CAN_HandleTypeDef* hcan,int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
	uint8_t tx_data[8];
  tx_data[0] = data1 >> 8;
  tx_data[1] = data1;
  tx_data[2] = data2 >> 8;
  tx_data[3] = data2;
  tx_data[4] = data3 >> 8;
  tx_data[5] = data3;
  tx_data[6] = data4 >> 8;
  tx_data[7] = data4;
  CAN_SendMsg(hcan,0,0x200,8,tx_data);
}

void CAN_Transceiver_Send(CAN_HandleTypeDef* hcan,int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
	uint8_t tx_data[8];
  tx_data[0] = data1 >> 8;
  tx_data[1] = data1;
  tx_data[2] = data2 >> 8;
  tx_data[3] = data2;
  tx_data[4] = data3 >> 8;
  tx_data[5] = data3;
  tx_data[6] = data4 >> 8;
  tx_data[7] = data4;
  CAN_SendMsg(hcan,0,0x222,8,tx_data);
}


