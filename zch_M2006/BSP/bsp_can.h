#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h" 

void CAN_FilterInit(CAN_HandleTypeDef* hcan);

uint8_t CAN_SendMsg(CAN_HandleTypeDef* hcan,uint8_t ide,uint32_t id,uint8_t len,uint8_t *data);

void CAN_M3508_Send(CAN_HandleTypeDef* hcan,int16_t data1, int16_t data2, int16_t data3, int16_t data4);

void CAN_Transceiver_Send(CAN_HandleTypeDef* hcan,int16_t data1, int16_t data2, int16_t data3, int16_t data4);

void CAN_GM6020_Send(CAN_HandleTypeDef* hcan,int16_t data1, int16_t data2, int16_t data3, int16_t data4);

void CAN_M2006_Send(CAN_HandleTypeDef* hcan,int16_t data1, int16_t data2, int16_t data3, int16_t data4);

#endif
