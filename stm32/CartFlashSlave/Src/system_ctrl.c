/*
 * system_ctrl.c
 *
 *  Created on: 14 Dec 2021
 *      Author: alf64
 */

#include "system_ctrl.h"
#include "core/pwr.h"
#include "core/rcc.h"
#include "core/flash.h"
#include "core/core.h"


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

void SystemClockCfg(void)
{
    pwr_set_voltage_output_scaling(PWR_VOS_SCALE2_MODE);

    /*
     * Our goal is to set SYSCLK to 84Mhz, clocked by the PLL.
     * --------------------------------------------------------
     * HSE on the PCB is: 8 MHz.
     * m = 8, meaning that PLL VCO is 1 MHz.
     * n = 336, meaning that inside PLL there is 336 MHz.
     * p = 4, which means 336 / 4 = 84 MHz, which can be used as a SYSCLK in the system, if PLLCLK is selected as source.
     * q = 7, which means 336 / 7 = 48 MHz for USB OTG is supplied.
     */
    core_ec_t ec = rcc_configure_and_enable_pll(
            RCC_PLLCLK_SRC_HSE,
            8,
            4,
            7,
            336);
    if(ec != CORE_EC_SUCCESS)
    {
        while(1){};
    }

    // increasing flash latency, as we are going to increase the CPU clock speed (HCLK)
    ec = flash_set_latency(2);
    if(ec != CORE_EC_SUCCESS)
    {
        while(1){};
    }

    /*
     * Our goal is to set:
     * AHB & HCLK to 84MHz.
     * APB1 to 48 MHz.
     * APB2 to 84MHz.
     * The clock source
     * ---------------------
     * SYSCLK is 84 MHz as set above via rcc_configure_and_enable_pll()
     * sysclk_to_ahb_div = 1, meaning that AHB and HCLK = 84 MHz.
     * ahb_to_apb1_div = 2, meaning that APB1 PCLK (peripheral) clock = 48 Mhz
     * (maximum possible for APB1, as per device datasheet 6.3.1 Operating Conditions)
     * ahb_to_apb2_div = 1, meaning that APB2 PCLK (peripheral) clock = 84 MHz
     * (maximum possible for APB2, as per device datasheet 6.3.1 Operating Conditions)
     * As as per figure 12 - Clock Tree, APBx Timer clocks multipliers are:
     * x1 if apbX_pre is 1
     * x2 if apbX_pre is !1
     * This means:
     * APB1 Timers clock multiplier will be x2, meaning APB1 Timers clock = 84 MHz.
     * APB2 Timers clock multiplier will be x1, meaning APB2 Timers clock = 84 MHz.
     */
    ec = rcc_configure_and_enable_main_clocks(
            RCC_SYSCLK_SRC_PLL,
            RCC_SYSCLK_TO_AHB_DIV_1,
            RCC_AHB_TO_APB1_DIV_2,
            RCC_AHB_TO_APB2_DIV_1);
    if(ec != CORE_EC_SUCCESS)
    {
        while(1){};
    }

    ec = core_systick_config_and_enable(84000, 0, 0);
    if(ec != CORE_EC_SUCCESS)
    {
        while(1){};
    }
}

void SystemInitHW(void)
{
    // init flash resources
    flash_icache_enable();
    flash_dcache_enable();
    flash_prefetch_enable();

    // set nvic priority grouping
    core_nvic_set_priogrp(CORE_NVIC_PRIOGRP_2_2);

    // enable clocks for used modules
    rcc_syscfg_clk_enable();
    rcc_pwr_clk_enable();
    rcc_gpio_clk_enable(GPIO_PORT_D);
}

void SystemInitGPIO(void)
{
    core_ec_t ec = CORE_EC_FAILED;

    gpio_pins_t pins_to_cfg = {0};
    pins_to_cfg.pins.pin12 = 1; // green LED
    pins_to_cfg.pins.pin13 = 1; // orange LED
    pins_to_cfg.pins.pin14 = 1; // red LED
    pins_to_cfg.pins.pin15 = 1; // blue LED
    ec = gpio_set_mode(
            GPIO_PORT_D,
            pins_to_cfg,
            GPIO_MODE_OUTPUT);
    HALT_ON_ERROR(ec);

    ec = gpio_set_out_type(
            GPIO_PORT_D,
            pins_to_cfg,
            GPIO_OTYPE_PUSH_PULL);
    HALT_ON_ERROR(ec);

    ec = gpio_set_out_speed(
            GPIO_PORT_D,
            pins_to_cfg,
            GPIO_OSPEED_LOW);
    HALT_ON_ERROR(ec);

    ec = gpio_set_pull_type(
            GPIO_PORT_D,
            pins_to_cfg,
            GPIO_PTYPE_NO_PULL);
    HALT_ON_ERROR(ec);

    // power on the green LED (PD12)
    pins_to_cfg.value = 0;
    pins_to_cfg.pins.pin12 = 1;
    gpio_pins_t pins_data = {0};
    pins_data.pins.pin12 = 1;
    ec = gpio_set_data(
            GPIO_PORT_D,
            pins_to_cfg,
            pins_data);
    HALT_ON_ERROR(ec);
}
