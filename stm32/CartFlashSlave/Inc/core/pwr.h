/*
 * pwr.h
 *
 *  Created on: 27 Oct 2021
 *      Author: alf64
 *
 * This file provides functions for Power Controller (PWR).
 */

#ifndef CORE_PWR_H_
#define CORE_PWR_H_

#include <core/ec.h>

#ifdef __cplusplus
 extern "C" {
#endif

/*
 * Possible values for VOS (Voltage Output Scaling)
 *
 * As per datasheet and operational conditions table (like typical frequencies and voltages),
 * this parameter lets you choose between:
 * 1. SCALE3 = Lower performance (smaller maximum frequencies on AHB and/or other buses) but greater power savings.
 * 2. SCALE2 = Greater performance, but lower power savings.
 */
typedef enum
{
    PWR_VOS_RESERVED1 = 0x0, // (reserved, scale3 mode selected)
    PWR_VOS_SCALE3_MODE = 0x1,
    PWR_VOS_SCALE2_MODE = 0x2,
    PWR_VOS_RESERVED2 = 0x3 // (reserved, scale2 mode selected)
}pwr_vos_t;

/*
 * @brief Sets the Voltage Output Scaling.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If unable to set the desired vos.
 * @retval CORE_EC_SUCCESS If succeeded to set the desired vos.
 */
core_ec_t pwr_set_voltage_output_scaling(pwr_vos_t vos);

#ifdef __cplusplus
}
#endif

#endif /* CORE_PWR_H_ */
