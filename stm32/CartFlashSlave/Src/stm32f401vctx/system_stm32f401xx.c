/*
  ******************************************************************************
  * @file    system_stm32f401xx.c
  * @author  alf64
  * @brief   System very low-level things.
  *
  *   This file provides:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f4xx.s" file.
  *
  ******************************************************************************
 */

#include <stm32f401vctx/system_stm32f4xx.h>
#include <stm32f401vctx/stm32f401xc.h>
#include "stm32f401vctx/stm32f401xx_hal.h"
#include "stm32f401vctx/stm32f4xx_hal_pwr.h"
#include "stm32f401vctx/stm32f4xx_hal_rcc.h"

/*
 * System main clock speed (in MHz). Default is equal to HSI_VALUE.
 * Should be updated each time there is a change made in clock configuration (enabling PLLs and such).
 */
uint32_t SystemCoreClock = HSI_VALUE;

//!< Frequency of the System Timer.
HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;

//!< Pre-emption priority of the System Timer.
uint32_t uwTickPrePrio = 0;

//!< Sub-priority of the System Timer.
uint32_t uwTickSubPrio = 0;

//!< NVIC Priority Grouping scheme.
uint32_t SystemPriorityGrouping = NVIC_PRIORITYGROUP_2;

void SystemInit(void)
{
#if (!defined(__SOFT_FP__) && defined(__ARM_FP))
    #warning "The project is compiling for an FPU. Maike sure you have initialized the FPU before use."
#endif

/* FPU settings in case it's enabled */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    #warning "FPU is present and used!"
    /*
     * SCB is a Cortex-M4 control block of registers, CPCAR is a Coprocessor Access Control Register from that pool
     * @see core_cm4.h
     */
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
#endif
}

void SystemClock_Config(void)
{
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2); // configures main internal regulator output voltage
    hal_status_t halstatus = HAL_RCC_OscConfig();
    if(halstatus != HAL_OK)
    {
        while(1){};
    }
    halstatus = HAL_RCC_ClockConfig();
    if(halstatus != HAL_OK)
    {
        while(1){};
    }
}

