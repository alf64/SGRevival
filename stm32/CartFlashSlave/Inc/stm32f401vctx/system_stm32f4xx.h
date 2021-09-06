/*
  ******************************************************************************
  * @file    system_stm32f401xx.h
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

#ifndef __SYSTEM_STM32F4XX_H
#define __SYSTEM_STM32F4XX_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
 extern "C" {
#endif 

//!< High Speed External oscillator frequency present on the system pcb.
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)8000000) /*!< Default value of the External oscillator in Hz */
#endif /* HSE_VALUE */

//!< High Speed Internal oscillator frequency present on the mcu.
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
* @brief Internal Low Speed oscillator (LSI) value.
*/
#if !defined  (LSI_VALUE)
#define LSI_VALUE  ((uint32_t)32000U)       /*!< LSI Typical Value in Hz*/
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                          The real value may vary depending on the variations
                                          in voltage and temperature.*/
/**
* @brief External Low Speed oscillator (LSE) value, present on system pcb.
*/
#if !defined  (LSE_VALUE)
#define LSE_VALUE  ((uint32_t)32768U)    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */

// Most common frequencies for System Timer.
typedef enum
{
HAL_TICK_FREQ_10HZ         = 100U,
HAL_TICK_FREQ_100HZ        = 10U,
HAL_TICK_FREQ_1KHZ         = 1U,
HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;


extern uint32_t SystemCoreClock; // holds current clock speed
extern HAL_TickFreqTypeDef uwTickFreq; // holds current System Timer frequency
extern uint32_t SystemPriorityGrouping; // holds current priority grouping
extern uint32_t uwTickPrePrio; // holds pre-emption priority of the System Timer
extern uint32_t uwTickSubPrio; // holds sub-emption priority of the System Timer

extern void SystemInit(void);
extern void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_STM32F4XX_H */
