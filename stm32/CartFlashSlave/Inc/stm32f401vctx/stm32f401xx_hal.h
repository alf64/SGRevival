/*
 * stm32f401xx_hal.h
 *
 *  Created on: 6 wrz 2021
 *      Author: alf64
 */

#ifndef STM32F401VCTX_STM32F401XX_HAL_H_
#define STM32F401VCTX_STM32F401XX_HAL_H_

// Macros for basic check nvic priorities values correctness.
#define IS_NVIC_PREEMPTION_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10U)
#define IS_NVIC_SUB_PRIORITY(PRIORITY)         ((PRIORITY) < 0x10U)

/**
  * @brief  HAL Status type.
  */
typedef enum _hal_status_t
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} hal_status_t;

/**
 * @brief Possible
 */
typedef enum
{
    //!< reserved, do not use
    NVIC_PRIORITYGROUP_RSVD1 = 0,
    NVIC_PRIORITYGROUP_RSVD2 = 1,
    NVIC_PRIORITYGROUP_RSVD3 = 2,
    //!< 4 bits for pre-emption priority, 0 bits for subpriority
    NVIC_PRIORITYGROUP_4 = 3,
    //!< 3 bits for pre-emption priority, 1 bit for subpriority
    NVIC_PRIORITYGROUP_3 = 4,
    //!< 2 bits for pre-emption priority, 2 bits for subpriority
    NVIC_PRIORITYGROUP_2 = 5,
    //!< 1 bit for pre-emption priority, 3 bits for subpriority
    NVIC_PRIORITYGROUP_1 = 6,
    //!< 0 bits for pre-emption priority, 4 bits for subpriority
    NVIC_PRIORITYGROUP_0 = 7
}nvic_prioritygroup_t;


//!< Inits needed hardware.
void HAL_InitHW();

//!< Inits System Timer.
hal_status_t HAL_InitTick(void);

#endif /* STM32F401VCTX_STM32F401XX_HAL_H_ */
