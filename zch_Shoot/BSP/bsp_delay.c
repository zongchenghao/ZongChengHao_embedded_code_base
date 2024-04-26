#include "bsp_delay.h"

static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: BSP_DelayInit
 * Description: 初始化延时参数
 * Return: 无
*************************************************/
void BSP_DelayInit(void)
{
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;
}

/*************************************************
 * Function: BSP_DelayUs
 * Description: BSP模块us级延时
 * Input: nus 需要延时的时间
 * Return: 无
*************************************************/
void BSP_DelayUs(uint16_t nus)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

/*************************************************
 * Function: BSP_DelayMs
 * Description: BSP模块ms级延时
 * Input: nms 需要延时的时间
 * Return: 无
*************************************************/
void BSP_DelayMs(uint16_t nms)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

/*************************************************
 * Function: BSP_GetTime_ms
 * Description: 获取系统运行时间 ms级
 * Return: 已运行的时间
*************************************************/
uint32_t BSP_GetTime_ms(void)
{
    return HAL_GetTick();
}
