#ifndef __BSP_DELAY_H__
#define __BSP_DELAY_H__

#include "main.h"

void BSP_DelayInit(void);
void BSP_DelayUs(uint16_t nus);
void BSP_DelayMs(uint16_t nms);
uint32_t BSP_GetTime_ms(void);


#endif
