#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "main.h"
#include "gpio.h"

typedef void (*GPIO_EXIT_Callback_t)(void);

typedef enum
{
    GPIO_EMPTY_DEV = 0,
    GPIO_OUTPUT_PP_DEV,
    GPIO_OUTPUT_OD_DEV,
    GPIO_INPUT_DEV,
    GPIO_PWM_DEV,
    GPIO_EXTI_DEV,
} GPIO_Device_e;

typedef struct
{
    void* handle;
    uint32_t value;
    GPIO_Device_e device;
    GPIO_EXIT_Callback_t callback;
} GPIO_Object_t;


#endif

