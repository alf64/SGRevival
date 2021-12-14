/*
 * system_ctrl.h
 *
 *  Created on: 14 Dec 2021
 *      Author: alf64
 */

#ifndef SYSTEM_CTRL_H_
#define SYSTEM_CTRL_H_

#ifdef __cplusplus
 extern "C" {
#endif

/*
 * Set to 1 if you plan to use libc along with malloc.
 * This will enable _sbrk() implementation from sysmem.c
 * file. Such implementation is needed for malloc()
 */
#define SYSTEM_CTRL_ENABLE_SBRK 0


//!< This function is called from startup file: startup_stm32f401vctx.s
void SystemInit(void);

//!< Configures clocks for the system.
void SystemClockCfg(void);

//!< Initializes hardware for the system.
void SystemInitHW(void);


#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_CTRL_H_ */
