/*
  ******************************************************************************
  * @file    stm32f401xx_hal.c
  * @author  alf64
  * @brief   General HAL functions for handling stm32f401xx.
  *
  ******************************************************************************
 */
#include "stm32f401vctx/stm32f401xc.h"
#include "stm32f401vctx/stm32f401xx_hal.h"
#include "stm32f401vctx/stm32f4xx_hal_flash.h"
#include "stm32f401vctx/stm32f4xx_hal_rcc.h"

/**
  * @brief  Sets the priority grouping field (preemption priority and subpriority)
  *         using the required unlock sequence.
  * @param  PriorityGroup The priority grouping bits length.
  *         This parameter can be one of the following values:
  *         @arg NVIC_PRIORITYGROUP_0: 0 bits for preemption priority
  *                                    4 bits for subpriority
  *         @arg NVIC_PRIORITYGROUP_1: 1 bits for preemption priority
  *                                    3 bits for subpriority
  *         @arg NVIC_PRIORITYGROUP_2: 2 bits for preemption priority
  *                                    2 bits for subpriority
  *         @arg NVIC_PRIORITYGROUP_3: 3 bits for preemption priority
  *                                    1 bits for subpriority
  *         @arg NVIC_PRIORITYGROUP_4: 4 bits for preemption priority
  *                                    0 bits for subpriority
  * @note   When the NVIC_PriorityGroup_0 is selected, IRQ preemption is no more possible.
  *         The pending IRQ priority will be managed only by the subpriority.
  * @retval None
  */
void HAL_NVIC_SetPriorityGrouping(nvic_prioritygroup_t PriorityGroup)
{
    switch(PriorityGroup)
    {
        case NVIC_PRIORITYGROUP_0:
        case NVIC_PRIORITYGROUP_1:
        case NVIC_PRIORITYGROUP_2:
        case NVIC_PRIORITYGROUP_3:
        case NVIC_PRIORITYGROUP_4:
        {
            NVIC_SetPriorityGrouping(PriorityGroup);
            SystemPriorityGrouping = PriorityGroup;
            break;
        }
        case NVIC_PRIORITYGROUP_RSVD1:
        case NVIC_PRIORITYGROUP_RSVD2:
        case NVIC_PRIORITYGROUP_RSVD3:
        default:
        {
            while(1){};
            break;
        }
    }
}


/**
  * @brief  Sets the priority of an interrupt.
  * @param  IRQn External interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f4xxxx.h))
  * @param  PreemptPriority The preemption priority for the IRQn channel.
  *         This parameter can be a value between 0 and 15
  *         A lower priority value indicates a higher priority
  * @param  SubPriority the subpriority level for the IRQ channel.
  *         This parameter can be a value between 0 and 15
  *         A lower priority value indicates a higher priority.
  * @retval None
  */
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{
  if(!IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority))
  {
      while(1){};
  }
  if(!IS_NVIC_SUB_PRIORITY(SubPriority))
  {
      while(1){};
  }

  uint32_t prioritygroup = NVIC_GetPriorityGrouping();

  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}


/**
  * @brief This function configures the source of the time base.
  *        The time source is configured  to have 1ms time base with a dedicated
  *        Tick interrupt priority.
  * @note In the default implementation, SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals.
  *       Care must be taken if HAL_Delay() is called from a peripheral ISR process,
  *       The SysTick interrupt must have higher priority (numerically lower)
  *       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  *       This function sets the priority of the System Timer to the highest.
  * @retval HAL status
  */
hal_status_t HAL_InitTick(void)
{
    // number of ticks needed for System Timer interrupt to occur every 1ms. For SystemCoreClock=16Mhz, ticks_interval==16000
    uint32_t ticks_interval = ((uint32_t)(SystemCoreClock / (uint32_t)(1000 / uwTickFreq)));

    // configure System Timer (enables System Timer Interrupt and sets some interrupt priority)
    uint32_t retval = SysTick_Config(ticks_interval);
    if(retval != 0)
    {
        return HAL_ERROR;
    }

    /* Change timer priority to our own. */
    HAL_NVIC_SetPriority(SysTick_IRQn, uwTickPrePrio, uwTickSubPrio);

  return HAL_OK;
}


void HAL_InitHW()
{
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_DATA_CACHE_ENABLE();
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

    __HAL_RCC_SYSCFG_CLK_ENABLE(); // enables clock for System Configuration Controller
    __HAL_RCC_PWR_CLK_ENABLE(); // enables clock for Power Controller / Power interface
}
