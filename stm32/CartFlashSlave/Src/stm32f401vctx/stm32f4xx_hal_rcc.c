/*
 * stm32f4xx_hal_rcc.c
 *
 *  Created on: 6 wrz 2021
 *      Author: alf64
 */

#include <stm32f401vctx/stm32f4xx_hal_rcc.h>
#include "stm32f401vctx/stm32f4xx_hal_flash.h"
#include "stm32f401vctx/stm32f401xx_hal.h"

hal_status_t HAL_RCC_OscConfig(void)
{
    // the main source for clocks is going to be HSE
    __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == 0){}; // wait for the HSE enablement flag

    if(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL) // if pll is not currently used as main clock source
    {
        __HAL_RCC_PLL_DISABLE();
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == 1){}; // wait for pll disablement

        __HAL_RCC_PLL_CONFIG(
                RCC_PLLSOURCE_HSE,
                8,
                336,
                RCC_PLLP_DIV4,
                7);

        __HAL_RCC_PLL_ENABLE();
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == 0){}; // wait for pll enablement

    }
    else
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}


hal_status_t HAL_RCC_ClockConfig(void)
{
    const uint32_t fl_lat_dst = FLASH_LATENCY_2;
    uint32_t fl_lat_read = __HAL_FLASH_GET_LATENCY();

    // set latency if dst latency greater than current
    if(fl_lat_dst > fl_lat_read)
    {
        __HAL_FLASH_SET_LATENCY(fl_lat_dst);
        while(__HAL_FLASH_GET_LATENCY() != fl_lat_dst){}; // wait for latency to be set
    }

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV16);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV16 << 3));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV1);

    // check if the pll is ready (should be enabled in HAL_RCC_OscConfig())
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == 0)
    {
        return HAL_ERROR;
    }

    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_PLLCLK);

    while(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL){}; // wait for pll to be set as system clock

    // set latency if dst latency lower than current
    if(fl_lat_dst < fl_lat_read)
    {
        __HAL_FLASH_SET_LATENCY(fl_lat_dst);
        while(__HAL_FLASH_GET_LATENCY() != fl_lat_dst){}; // wait for latency to be set
    }

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV2);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV1 << 3));

    SystemCoreClock = 84000000UL; // should be 84 MHz now
    hal_status_t halstatus = HAL_InitTick();

    return halstatus;
}

