/*
 * flash.h
 *
 *  Created on: 15 Oct 2021
 *      Author: alf64
 *
 * Provides functionality for Embedded FLASH memory interface.
 */

#ifndef CORE_FLASH_H_
#define CORE_FLASH_H_

#include <core/ec.h>

#ifdef __cplusplus
 extern "C" {
#endif

/*
 * Enables flash instruction cache.
 */
void flash_icache_enable(void);

/*
 * Disables flash instruction cache.
 */
void flash_icache_disable(void);

/*
 * Enables flash data cache.
 */
void flash_dcache_enable(void);

/*
 * Disables flash data cache.
 */
void flash_dcache_disable(void);

/*
 * Enables flash prefetch buffer.
 * @note
 * The prefetch buffer must be disabled when the supply voltage is below 2.1 V
 */
void flash_prefetch_enable(void);

/*
 * Disables flash prefetch buffer.
 */
void flash_prefetch_disable(void);

/*
 * Gets flash latency.
 * @returns uint8_t - flash latency
 * @retval 0 - 15 flash latency (0 - no latency, 15 - fifteen cpu cycles wait states)
 */
uint8_t flash_get_latency(void);

/*
 * Sets flash latency.
 *
 * @details
 * To correctly read data from Flash memory, the number of wait states (LATENCY) must be
 * correctly programmed in the Flash access control register (FLASH_ACR) according to the
 * frequency of the CPU clock (HCLK) and the supply voltage of the device.
 *
 * After reset, the CPU clock frequency is 16 MHz and 0 wait state (WS) (no latency) is configured by default.
 * If you are going to increase the CPU clock (HCLK), first set the new latency, then set the new clock.
 * If you are going to decrease the CPU clock (HCLK), first set the new clock, then set the new latency.
 *
 * From Reference Manual - Table 6.: Recommended latency settings for Voltage range 2.7 V - 3.6 V:
 * latency = 0, when 0 < HCLK <= 30
 * latency = 1, when 30 < HCLK <= 60
 * latency = 2, when 60 < HCLK <= 84
 *
 * @param latency Flash latency to set, from range: 0 - 15
 * (0 - no latency, 15 - fifteen cpu cycles wait states)
 * Maximum latency should be 5 for stm32f401, that's what accepted by this function.
 * @returns core_ec_t - error code
 * @retval CORE_EC_FAILED If failed to set latency.
 * @retval CORE_EC_SUCCESS If succeeded to set latency.
 */
core_ec_t flash_set_latency(uint8_t latency);

#ifdef __cplusplus
}
#endif

#endif /* CORE_FLASH_H_ */
