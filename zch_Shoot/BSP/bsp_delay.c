#include "bsp_delay.h"

static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/
/*************************************************
 * Function: BSP_DelayInit
 * Description: ��ʼ����ʱ����
 * Return: ��
*************************************************/
void BSP_DelayInit(void)
{
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;
}

/*************************************************
 * Function: BSP_DelayUs
 * Description: BSPģ��us����ʱ
 * Input: nus ��Ҫ��ʱ��ʱ��
 * Return: ��
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
 * Description: BSPģ��ms����ʱ
 * Input: nms ��Ҫ��ʱ��ʱ��
 * Return: ��
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
 * Description: ��ȡϵͳ����ʱ�� ms��
 * Return: �����е�ʱ��
*************************************************/
uint32_t BSP_GetTime_ms(void)
{
    return HAL_GetTick();
}
