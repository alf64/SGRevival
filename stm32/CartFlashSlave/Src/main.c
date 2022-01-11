/**
 ******************************************************************************
 * @file           : main.c
 * @author         : alf64
 * @brief          : Main program body
 *
 *
 * STM32F401VCT6U is used in this application.
 *
 * STM32CubeIDE and STM32F4xB/C specifies the following for this uC:
 * RAM is 64kB, address range: 0x20000000 - 0x2000FFFF
 * FLASH is 256kB, address range:: 0x08000000 - 0x0803FFFF
 * STM32 peripherals address range is: 0x40000000 - 0x5fffffff (stm32 specific registers)
 * @see stm32f401xc.h for the details regarding stm32 registers and addresses.
 * ARM Cortex-M4 peripherals address range is: 0xE0000000 - 0xFFFFFFFF (ARM specific registers)
 * @see core_cm4.h for the details regarding ARM Cortex-M4 registers.
 *
 * FPU has been explicitly disabled in compiler settings (FPU set to none), since
 * it's not used in this project.
 *
 ****************************************************************************** */

#include <stdint.h>
#include <system_ctrl.h>

int main(void)
{
    SystemInit();
    SystemInitHW();
    SystemClockCfg();
    SystemInitGPIO();

    /* Loop forever */
	for(;;);

	return 0;
}
