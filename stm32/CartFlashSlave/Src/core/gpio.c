/*
 * gpio.c
 *
 *  Created on: 6 wrz 2021
 *      Author: alf64
 */

#include "stm32f401vctx/stm32f4xx_hal_rcc.h"

void MX_GPIO_Init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    //TODO: add gpio init (mode, state, ...)
}
