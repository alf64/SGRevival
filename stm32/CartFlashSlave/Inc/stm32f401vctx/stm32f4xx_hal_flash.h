/*
 * stm32f4xx_hal_flash.h
 *
 *  Created on: 12 sie 2021
 *      Author: alf64
 */

#ifndef STM32F401VCTX_STM32F4XX_HAL_FLASH_H_
#define STM32F401VCTX_STM32F4XX_HAL_FLASH_H_

#include "stm32f401xc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Set the FLASH Latency.
  * @param  __LATENCY__ FLASH Latency
  *         The value of this parameter depend on device used within the same series
  * @retval none
  */
#define __HAL_FLASH_SET_LATENCY(__LATENCY__) (*(__IO uint8_t *)(FLASH_ACR_BYTE0_ADDRESS) = (uint8_t)(__LATENCY__))

/**
  * @brief  Get the FLASH Latency.
  * @retval FLASH Latency
  *          The value of this parameter depend on device used within the same series
  */
#define __HAL_FLASH_GET_LATENCY()     (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY))

/**
* @brief  Enable the FLASH prefetch buffer.
* @retval none
*/
#define __HAL_FLASH_PREFETCH_BUFFER_ENABLE()  (FLASH->ACR |= FLASH_ACR_PRFTEN)

/**
* @brief  Disable the FLASH prefetch buffer.
* @retval none
*/
#define __HAL_FLASH_PREFETCH_BUFFER_DISABLE()   (FLASH->ACR &= (~FLASH_ACR_PRFTEN))

/**
* @brief  Enable the FLASH instruction cache.
* @retval none
*/
#define __HAL_FLASH_INSTRUCTION_CACHE_ENABLE()  (FLASH->ACR |= FLASH_ACR_ICEN)

/**
* @brief  Disable the FLASH instruction cache.
* @retval none
*/
#define __HAL_FLASH_INSTRUCTION_CACHE_DISABLE()   (FLASH->ACR &= (~FLASH_ACR_ICEN))

/**
* @brief  Enable the FLASH data cache.
* @retval none
*/
#define __HAL_FLASH_DATA_CACHE_ENABLE()  (FLASH->ACR |= FLASH_ACR_DCEN)

/**
* @brief  Disable the FLASH data cache.
* @retval none
*/
#define __HAL_FLASH_DATA_CACHE_DISABLE()   (FLASH->ACR &= (~FLASH_ACR_DCEN))

/*
 * Possible flash latencies.
 */
#define FLASH_LATENCY_0                FLASH_ACR_LATENCY_0WS   /*!< FLASH Zero Latency cycle      */
#define FLASH_LATENCY_1                FLASH_ACR_LATENCY_1WS   /*!< FLASH One Latency cycle       */
#define FLASH_LATENCY_2                FLASH_ACR_LATENCY_2WS   /*!< FLASH Two Latency cycles      */
#define FLASH_LATENCY_3                FLASH_ACR_LATENCY_3WS   /*!< FLASH Three Latency cycles    */
#define FLASH_LATENCY_4                FLASH_ACR_LATENCY_4WS   /*!< FLASH Four Latency cycles     */
#define FLASH_LATENCY_5                FLASH_ACR_LATENCY_5WS   /*!< FLASH Five Latency cycles     */
#define FLASH_LATENCY_6                FLASH_ACR_LATENCY_6WS   /*!< FLASH Six Latency cycles      */
#define FLASH_LATENCY_7                FLASH_ACR_LATENCY_7WS   /*!< FLASH Seven Latency cycles    */

#ifdef __cplusplus
}
#endif

#endif /* STM32F401VCTX_STM32F4XX_HAL_FLASH_H_ */
