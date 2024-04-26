#include "bsp_gpio.h"

static GPIO_Object_t* m_objects[14];


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (uint8_t i=0; i < 14; i++)
    {
        if (m_objects[i]->value == GPIO_Pin)
        {
            if (m_objects[i]->device == GPIO_EXTI_DEV)
                m_objects[i]->callback();
        }
    }
}
