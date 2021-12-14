/*
 * core.h
 *
 *  Created on: 25 paź 2021
 *      Author: alf64
 *
 * Provides functionality specific to the ARM core,
 * i.e.: System Timer (SysTick), Interrupts (NVIC), ...
 *
 * @see ARMv7-M architecture manual for reference and details.
 */

#ifndef CORE_CORE_H_
#define CORE_CORE_H_

#include <core/ec.h>
#include <stm32f401vctx/stm32f401xc.h>

#ifdef __cplusplus
 extern "C" {
#endif

/* ---------- Functionality for Nested Vectored Interrupt Controller (NVIC) ---------- */

//!<
/*
 * Possible nvic priority gropupings for stm32f401.
 * Functions that operate on this will not function correctly if
 * there is any other __NVIC_PRIO_BITS (different core, as it depends on it).
 * For other cores, you will have to add support in the future.
 */
#if defined(__NVIC_PRIO_BITS) && (__NVIC_PRIO_BITS == 4)
typedef enum
{
    //!< reserved, do not use
    CORE_NVIC_PRIOGRP_RSVD1 = 0,
    CORE_NVIC_PRIOGRP_RSVD2 = 1,
    CORE_NVIC_PRIOGRP_RSVD3 = 2,
    //!< 0 bits for preemption priority, 4 bits for subpriority
    CORE_NVIC_PRIOGRP_0_4 = 7,
    //!< 1 bit for preemption priority, 3 bits for subpriority
    CORE_NVIC_PRIOGRP_1_3 = 6,
    //!< 2 bits for preemption priority, 2 bits for subpriority
    CORE_NVIC_PRIOGRP_2_2 = 5,
    //!< 3 bits for preemption priority, 1 bit for subpriority
    CORE_NVIC_PRIOGRP_3_1 = 4,
    //!< 4 bits for preemption priority, 0 bits for subpriority
    CORE_NVIC_PRIOGRP_4_0 = 3
}core_nvic_priogrp_t;
#else
#error "core.h does not support __NVIC_PRIO_BITS different than 4 at the moment!"
#endif


/*
 * @brief Sets priority grouping scheme for NVIC.
 *
 * @details
 * This function sets NVIC to predefined interrupt priority grouping.
 * Possible priority groupings depends on the uC and its ARM core,
 * since these are closely coupled.
 *
 * I.e. for STM32F401 there are 4 bits used for priority (meaning there are 16 levels of priority)
 * and this priority can be divided into: preemption (group) priority and subpriority.
 *
 * Any IRQ (like IRQ for the UART, EXTI, SPI, Timers, ...) can then have a priority assigned
 * (preemption and subpriority).
 *
 * IRQs within the same preemption priority do not preempt each other.
 * This means that during one IRQ execution, new IRQ that occurs cannot break the current IRQ execution.
 * Instead it will have to wait until current IRQ execution ends.
 * The lower the subpriority, the greater the priority of the IRQ, giving it the precedence for the next IRQ execution.
 *
 * IRQs from different preemption priorities preempt each other.
 * This means that during one IRQ execution, new IRQ that occurs can break the current IRQ execution.
 * (if the new IRQ has lower preemption priority).
 * When the new IRQ ends, execution goes back to the previous IRQ that was under execution.
 * The lower the preemption priority, the greater the priority of the IRQ, giving it the precedence for the next IRQ execution.
 *
 * @param priority_grouping - desired priority grouping scheme.
 *         This parameter can be one of the following values:
 *         @arg CORE_NVIC_PRIOGRP_0_4: 0 bits for preemption priority
 *                                    4 bits for subpriority
 *         @arg CORE_NVIC_PRIOGRP_1_3: 1 bits for preemption priority
 *                                    3 bits for subpriority
 *         @arg CORE_NVIC_PRIOGRP_2_2: 2 bits for preemption priority
 *                                    2 bits for subpriority
 *         @arg CORE_NVIC_PRIOGRP_3_1: 3 bits for preemption priority
 *                                    1 bits for subpriority
 *         @arg CORE_NVIC_PRIOGRP_4_0: 4 bits for preemption priority
 *                                    0 bits for subpriority
 *
 * The group priorities of Reset, NMI and HardFault are -3, -2, and -1 respectively, regardless of the value of PRIGROUP.
 */
core_ec_t core_nvic_set_priogrp(core_nvic_priogrp_t priority_grouping);

/*
 * @brief Sets preemption priority and subpriority for the given interrupt / exception.
 *
 * @param irq An interrupt/exception for which the priorities shall be set.
 * @param preemption (group) priority to set for the irq.
 * @param subpriority to set for the irq.
 *
 * @returns core_ec_t
 * @retval CORE_EC_SUCCES Means the function succeeded.
 * @retval CORE_EC_FAILED Means the function failed.
 */
core_ec_t core_nvic_set_priority(IRQn_Type irq, uint32_t preeprio, uint32_t subprio);

/* ---------- End of functionality for Nested Vectored Interrupt Controller (NVIC) ---------- */


/* ---------- Functionality for SysTick Timer. ---------- */

/*
 * @brief Configures and enables ARM Systick Timer.
 *
 * @details
 * This function sets the Systick Timer interval, sets the Systick Interrupt (SysTick_IRQn) priority
 * and enables this timer along with its interrupt.
 *
 * @note: this function is based on SysTick_Config() from core_cm4.h
 * It's just own implementation.
 *
 * @param interval Number of clock ticks needed for the Systick Timer Interrupt to occur.
 * Systick Timer is being clocked with the same frequency as core (HCLK).
 * Example: if you have HCLK = 84 MHz (84M ticks per second), and you want SysTick Timer to give you interrupts
 * every 1ms, you should set interval to 84000.
 * @param preeprio - Preemption priority for the SysTick Timer Interrupt.
 * @param subprio - Subpriority for the SysTick Timer Interrupt.
 * @returns core_ec_t
 * @retval CORE_EC_SUCCES Means the function succeeded.
 * @retval CORE_EC_FAILED Means the function failed.
 */
core_ec_t core_systick_config_and_enable(uint32_t interval, uint8_t preeprio, uint8_t subprio);

/* ---------- End of functionality for SysTick Timer ---------- */

#ifdef __cplusplus
}
#endif

#endif /* CORE_CORE_H_ */
