/*
 * rcc.c
 *
 *  Created on: 18 paź 2021
 *      Author: alf64
 */

#include <core/rcc.h>
#include <stm32f401vctx/stm32f401xc.h>

/*
 * Nomenclature:
 * RO - Read Only
 * RW - Read-Write
 * RC_W1 - Read-Clear-Write-1 (writing 0 has no effect)
 * RC_W0 - Read-Clear-Write-0 (writing 1 has no effect)
 * RS - Read-Set (Writing 0 has no effect)
 * W - Only write is allowed. Reading will just return the reset value.
 * RT_W - Read-Only Write-Trigger. Software can read this bit. Writing '0' or '1' triggers an event but has
 * no effect on the bit value.
 */

/*
 * ---------- RCC registers and bits definitions ----------
 */
typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Set and cleared by software.
        Set by hardware to force the HSI oscillator ON when leaving the Stop or Standby mode or in
        case of a failure of the HSE oscillator used directly or indirectly as the system clock. This bit
        cannot be cleared if the HSI is used directly or indirectly as the system clock.
        0: HSI oscillator OFF
        1: HSI oscillator ON
         */
        uint32_t hsion : 1; // [RW] HSI (Internal high-speed) clock enable
        /*
         * Set by hardware to indicate that the HSI oscillator is stable. After the HSION bit is cleared,
        HSIRDY goes low after 6 HSI clock cycles.
        0: HSI oscillator not ready
        1: HSI oscillator ready
         */
        uint32_t hsirdy : 1; // [RO] HSI (Internal high-speed) clock ready flag
        uint32_t rsvd2 : 1; // Reserved
        /*
         * These bits provide an additional user-programmable trimming value that is added to the
        HSICAL[7:0] bits. It can be programmed to adjust to variations in voltage and temperature
        that influence the frequency of the internal HSI RC.
         */
        uint32_t hsitrim : 5; // [RW] Internal high-speed clock trimming
        /*
         * These bits are initialized automatically at startup.
         */
        uint32_t hsical : 8; // [RO] Internal high-speed clock calibration
        /*
         * Set and cleared by software.
        Cleared by hardware to stop the HSE oscillator when entering Stop or Standby mode. This
        bit cannot be reset if the HSE oscillator is used directly or indirectly as the system clock.
        0: HSE oscillator OFF
        1: HSE oscillator ON
         */
        uint32_t hseon : 1; // [RW] HSE (High speed external) clock enable
        /*
         * Set by hardware to indicate that the HSE oscillator is stable. After the HSEON bit is cleared,
        HSERDY goes low after 6 HSE oscillator clock cycles.
        0: HSE oscillator not ready
        1: HSE oscillator ready
         */
        uint32_t hserdy : 1; // [RO] HSE (High speed external) clock ready flag
        /*
         * Set and cleared by software to bypass the oscillator with an external clock. The external
        clock must be enabled with the HSEON bit, to be used by the device.
        The HSEBYP bit can be written only if the HSE oscillator is disabled.
        0: HSE oscillator not bypassed
        1: HSE oscillator bypassed with an external clock
         */
        uint32_t hsebyp : 1; // [RW] HSE (High speed external) clock bypass
        /*
         * Set and cleared by software to enable the clock security system. When CSSON is set, the
        clock detector is enabled by hardware when the HSE oscillator is ready, and disabled by
        hardware if an oscillator failure is detected.
        0: Clock security system OFF (Clock detector OFF)
        1: Clock security system ON (Clock detector ON if HSE oscillator is stable, OFF if not)
         */
        uint32_t csson : 1; // [RW] Clock security system enable
        uint32_t rsvd20_23 : 4; // Reserved
        /*
         * Set and cleared by software to enable PLL.
        Cleared by hardware when entering Stop or Standby mode. This bit cannot be reset if PLL
        clock is used as the system clock.
        0: PLL OFF
        1: PLL ON
         */
        uint32_t pllon : 1; // [RW] Main PLL enable
        /*
         * Set by hardware to indicate that PLL is locked.
        0: PLL unlocked
        1: PLL locked
         */
        uint32_t pllrdy : 1; // [RO] Main PLL clock ready flag
        /*
         * Set and cleared by software to enable PLLI2S.
        Cleared by hardware when entering Stop or Standby mode.
        0: PLLI2S OFF
        1: PLLI2S ON
         */
        uint32_t plli2son : 1; // [RW] PLLI2S enable
        /*
         * Set by hardware to indicate that the PLLI2S is locked.
        0: PLLI2S unlocked
        1: PLLI2S locked
         */
        uint32_t plli2srdy : 1; // [RO] PLLI2S clock ready flag
        uint32_t rsvd28_31 : 4; // Reserved
    }bits;
} rcc_reg_cr_t;

/*
 *
 */
typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Set and cleared by software to divide the PLL and PLLI2S input clock before the VCO.
        These bits can be written only when the PLL and PLLI2S are disabled.
        Caution: The software has to set these bits correctly to ensure that the VCO input frequency
        ranges from 1 to 2 MHz. It is recommended to select a frequency of 2 MHz to limit
        PLL jitter.
        VCO input frequency = PLL input clock frequency / PLLM with 2 ≤PLLM ≤63
        000000: PLLM = 0, wrong configuration
        000001: PLLM = 1, wrong configuration
        000010: PLLM = 2
        000011: PLLM = 3
        000100: PLLM = 4
        ...
        111110: PLLM = 62
        111111: PLLM = 63
         */
        uint32_t pllm : 6; // [RW] Division factor for the main PLL (PLL) and audio PLL (PLLI2S) input clock
        /*
         * Set and cleared by software to control the multiplication factor of the VCO. These bits can
        be written only when PLL is disabled. Only half-word and word accesses are allowed to
        write these bits.
        Caution: The software has to set these bits correctly to ensure that the VCO output
        frequency is between 192 and 432 MHz. (check also Section 6.3.20: RCC PLLI2S
        configuration register (RCC_PLLI2SCFGR))
        VCO output frequency = VCO input frequency × PLLN with 192 ≤PLLN ≤432
        000000000: PLLN = 0, wrong configuration
        000000001: PLLN = 1, wrong configuration
        ...
        ...
        110110000: PLLN = 432
        110110001: PLLN = 433, wrong configuration
        ...
        111111111: PLLN = 511, wrong configuration
         */
        uint32_t plln : 9; // [RW] Main PLL (PLL) multiplication factor for VCO
        uint32_t rsvd_15 : 1;
        /*
         * Set and cleared by software to control the frequency of the general PLL output clock. These
        bits can be written only if PLL is disabled.
        Caution: The software has to set these bits correctly not to exceed 84 MHz on this domain.
        PLL output clock frequency = VCO frequency / PLLP with PLLP = 2, 4, 6, or 8
        00: PLLP = 2
        01: PLLP = 4
        10: PLLP = 6
        11: PLLP = 8
         */
        uint32_t pllp : 2; // [RW] Main PLL (PLL) division factor for main system clock
        uint32_t rsvd_18_21 : 4;
        /*
         * Set and cleared by software to select PLL and PLLI2S clock source. This bit can be written
        only when PLL and PLLI2S are disabled.
        0: HSI clock selected as PLL and PLLI2S clock entry
        1: HSE oscillator clock selected as PLL and PLLI2S clock entry
         */
        uint32_t pllsrc : 1; // [RW] Main PLL(PLL) and audio PLL (PLLI2S) entry clock source
        uint32_t rsvd23 : 1;
        /*
         * Set and cleared by software to control the frequency of USB OTG FS clock, the random
        number generator clock and the SDIO clock. These bits should be written only if PLL is
        disabled.
        Caution: The USB OTG FS requires a 48 MHz clock to work correctly. The SDIO and the
        random number generator need a frequency lower than or equal to 48 MHz to work
        correctly.
        USB OTG FS clock frequency = VCO frequency / PLLQ with 2 ≤PLLQ ≤15
        0000: PLLQ = 0, wrong configuration
        0001: PLLQ = 1, wrong configuration
        0010: PLLQ = 2
        0011: PLLQ = 3
        0100: PLLQ = 4
        ...
        1111: PLLQ = 15
         */
        uint32_t pllq : 4; // [RW] Main PLL (PLL) division factor for USB OTG FS, SDIO and random number generator clocks.
        uint32_t rsvd28_31 : 4; // Reserved
    }bits;
} rcc_reg_pllcfgr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Set and cleared by software to select the system clock source.
        Set by hardware to force the HSI selection when leaving the Stop or Standby mode or in
        case of failure of the HSE oscillator used directly or indirectly as the system clock.
        00: HSI oscillator selected as system clock
        01: HSE oscillator selected as system clock
        10: PLL selected as system clock
        11: not allowed
         */
        uint32_t sw : 2; // [RW] System Clock Switch
        /*
         * Set and cleared by hardware to indicate which clock source is used as the system clock.
        00: HSI oscillator used as the system clock
        01: HSE oscillator used as the system clock
        10: PLL used as the system clock
        11: not applicable
         */
        uint32_t sws : 2; // [RO] System Clock Switch status
        /*
         * Set and cleared by software to control AHB clock division factor.
        Caution: The clocks are divided with the new prescaler factor from 1 to 16 AHB cycles after
        HPRE write.
        0xxx: system clock not divided
        1000: system clock divided by 2
        1001: system clock divided by 4
        1010: system clock divided by 8
        1011: system clock divided by 16
        1100: system clock divided by 64
        1101: system clock divided by 128
        1110: system clock divided by 256
        1111: system clock divided by 512
         */
        uint32_t hpre : 4; // [RW] AHB prescaler
        uint32_t rsvd8_9 : 2; // Reserved
        /*
         * Set and cleared by software to control APB low-speed clock division factor.
        Caution: The software has to set these bits correctly not to exceed 42 MHz on this domain.
        The clocks are divided with the new prescaler factor from 1 to 16 AHB cycles after
        PPRE1 write.
        0xx: AHB clock not divided
        100: AHB clock divided by 2
        101: AHB clock divided by 4
        110: AHB clock divided by 8
        111: AHB clock divided by 16
         */
        uint32_t ppre1 : 3; // [RW] APB Low speed prescaler (APB1)
        /*
         * Set and cleared by software to control APB high-speed clock division factor.
        Caution: The software has to set these bits correctly not to exceed 84 MHz on this domain.
        The clocks are divided with the new prescaler factor from 1 to 16 AHB cycles after
        PPRE2 write.
        0xx: AHB clock not divided
        100: AHB clock divided by 2
        101: AHB clock divided by 4
        110: AHB clock divided by 8
        111: AHB clock divided by 16
         */
        uint32_t ppre2 : 3; // [RW]  APB High speed prescaler (APB2)
        /*
         * Set and cleared by software to divide the HSE clock input clock to generate a 1 MHz clock
        for RTC.
        Caution: The software has to set these bits correctly to ensure that the clock supplied to the
        RTC is 1 MHz. These bits must be configured if needed before selecting the RTC
        clock source.
        00000: no clock
        00001: no clock
        00010: HSE/2
        00011: HSE/3
        00100: HSE/4
        ...
        11110: HSE/30
        11111: HSE/31
         */
        uint32_t rtcpre : 5; // [RW] HSE division factor for RTC clock
        /*
         * Set and cleared by software. Clock source selection may generate glitches on MCO1. It is
        highly recommended to configure these bits only after reset before enabling the external
        oscillators and PLL.
        00: HSI clock selected
        01: LSE oscillator selected
        10: HSE oscillator clock selected
        11: PLL clock selected
         */
        uint32_t mco1 : 2; // [RW] Microcontroller clock output 1
        /*
         * Set and cleared by software. This bit allows to select the I2S clock source between the
        PLLI2S clock and the external clock. It is highly recommended to change this bit only after
        reset and before enabling the I2S module.
        0: PLLI2S clock used as I2S clock source
        1: External clock mapped on the I2S_CKIN pin used as I2S clock source
         */
        uint32_t i2ssrc : 1; // [RW] I2S clock selection
        /*
         * Set and cleared by software to configure the prescaler of the MCO1. Modification of this
        prescaler may generate glitches on MCO1. It is highly recommended to change this
        prescaler only after reset before enabling the external oscillators and the PLL.
        0xx: no division
        100: division by 2
        101: division by 3
        110: division by 4
        111: division by 5
         */
        uint32_t mco1pre : 3; // [RW] MCO1 prescaler
        /*
         * Set and cleared by software to configure the prescaler of the MCO2. Modification of this
        prescaler may generate glitches on MCO2. It is highly recommended to change this
        prescaler only after reset before enabling the external oscillators and the PLLs.
        0xx: no division
        100: division by 2
        101: division by 3
        110: division by 4
        111: division by 5
         */
        uint32_t mco2pre : 3; // [RW] MCO2 prescaler
        /*
         * Set and cleared by software. Clock source selection may generate glitches on MCO2. It is
        highly recommended to configure these bits only after reset before enabling the external
        oscillators and the PLLs.
        00: System clock (SYSCLK) selected
        01: PLLI2S clock selected
        10: HSE oscillator clock selected
        11: PLL clock selected
         */
        uint32_t mco2 : 2; // [RW] Microcontroller clock output 1
    }bits;
} rcc_reg_cfgr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Set by hardware when the internal low speed clock becomes stable and LSIRDYDIE is set.
        Cleared by software setting the LSIRDYC bit.
        0: No clock ready interrupt caused by the LSI oscillator
        1: Clock ready interrupt caused by the LSI oscillator
         */
        uint32_t lsirdyf : 1; // [RO] LSI ready interrupt flag.
        /*
         * Set by hardware when the External Low Speed clock becomes stable and LSERDYDIE is
        set.
        Cleared by software setting the LSERDYC bit.
        0: No clock ready interrupt caused by the LSE oscillator
        1: Clock ready interrupt caused by the LSE oscillator
         */
        uint32_t lserdyf : 1; // [RO] LSE ready interrupt flag.
        /*
         * Set by hardware when the Internal High Speed clock becomes stable and HSIRDYDIE is
        set.
        Cleared by software setting the HSIRDYC bit.
        0: No clock ready interrupt caused by the HSI oscillator
        1: Clock ready interrupt caused by the HSI oscillator
         */
        uint32_t hsirdyf : 1; // [RO] HSI ready interrupt flag.
        /*
         * Set by hardware when External High Speed clock becomes stable and HSERDYDIE is set.
        Cleared by software setting the HSERDYC bit.
        0: No clock ready interrupt caused by the HSE oscillator
        1: Clock ready interrupt caused by the HSE oscillator
         */
        uint32_t hserdyf : 1; // [RO] HSE ready interrupt flag.
        /*
         * Set by hardware when PLL locks and PLLRDYDIE is set.
        Cleared by software setting the PLLRDYC bit.
        0: No clock ready interrupt caused by PLL lock
        1: Clock ready interrupt caused by PLL lock
         */
        uint32_t pllrdyf : 1; // [RO] Main PLL ready interrupt flag.
        /*
         * Set by hardware when the PLLI2S locks and PLLI2SRDYDIE is set.
        Cleared by software setting the PLLRI2SDYC bit.
        0: No clock ready interrupt caused by PLLI2S lock
        1: Clock ready interrupt caused by PLLI2S lock
         */
        uint32_t plli2srdyf : 1; // [RO] PLLI2S ready interrupt flag.
        uint32_t rsvd6 : 1; // Reserved
        /*
         * Set by hardware when a failure is detected in the HSE oscillator.
        Cleared by software setting the CSSC bit.
        0: No clock security interrupt caused by HSE clock failure
        1: Clock security interrupt caused by HSE clock failure
         */
        uint32_t cssf : 1; // [RO] Clock security system interrupt flag.
        /*
         * Set and cleared by software to enable/disable interrupt caused by LSI oscillator
        stabilization.
        0: LSI ready interrupt disabled
        1: LSI ready interrupt enabled
         */
        uint32_t lsirdyie : 1; // [RW] LSI ready interrupt enable.
        /*
         * Set and cleared by software to enable/disable interrupt caused by the LSE oscillator
        stabilization.
        0: LSE ready interrupt disabled
        1: LSE ready interrupt enabled
         */
        uint32_t lserdyie : 1; // [RW] LSE ready interrupt enable.
        /*
         * Set and cleared by software to enable/disable interrupt caused by the HSI oscillator
        stabilization.
        0: HSI ready interrupt disabled
        1: HSI ready interrupt enabled
         */
        uint32_t hsirdyie : 1; // [RW] HSI ready interrupt enable.
        /*
         * Set and cleared by software to enable/disable interrupt caused by the HSE oscillator
        stabilization.
        0: HSE ready interrupt disabled
        1: HSE ready interrupt enabled
         */
        uint32_t hserdyie : 1; // [RW] HSE ready interrupt enable.
        /*
         * Set and cleared by software to enable/disable interrupt caused by PLL lock.
        0: PLL lock interrupt disabled
        1: PLL lock interrupt enabled
         */
        uint32_t pllrdyie : 1; // [RW] Main PLL ready interrupt enable.
        /*
         * Set and cleared by software to enable/disable interrupt caused by PLLI2S lock.
        0: PLLI2S lock interrupt disabled
        1: PLLI2S lock interrupt enabled
         */
        uint32_t plli2srdyie : 1; // [RW] PLLI2S ready interrupt enable.
        uint32_t rsvd14_15 : 2; // Reserved
        /*
         * This bit is set by software to clear the LSIRDYF flag.
        0: No effect
        1: LSIRDYF cleared
         */
        uint32_t lsirdyc : 1; // [W] LSI ready interrupt clear
        /*
         * This bit is set by software to clear the LSERDYF flag.
        0: No effect
        1: LSERDYF cleared
         */
        uint32_t lserdyc : 1; // [W] LSE ready interrupt clear
        /*
        This bit is set software to clear the HSIRDYF flag.
        0: No effect
        1: HSIRDYF cleared
         */
        uint32_t hsirdyc : 1; // [W] HSI ready interrupt clear
        /*
         * This bit is set by software to clear the HSERDYF flag.
        0: No effect
        1: HSERDYF cleared
         */
        uint32_t hserdyc : 1; // [W] HSE ready interrupt clear
        /*
         * This bit is set by software to clear the PLLRDYF flag.
        0: No effect
        1: PLLRDYF cleared
         */
        uint32_t pllrdyc : 1; // [W] Main PLL ready interrupt clear
        /*
         * This bit is set by software to clear the PLLI2SRDYF flag.
        0: No effect
        1: PLLI2SRDYF cleared
         */
        uint32_t plli2srdyc : 1; // [W] PLLI2S ready interrupt clear
        uint32_t rsvd22 : 1; // Reserved
        /*
         * This bit is set by software to clear the CSSF flag.
        0: No effect
        1: Clear CSSF flag
         */
        uint32_t cssc : 1; // [W] Clock security system interrupt clear
        uint32_t rsvd24_31 : 8; // Reserved
    }bits;
} rcc_reg_cir_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t gpioarst : 1; // [RW] IO port A reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t gpiobrst : 1; // [RW] IO port B reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t gpiocrst : 1; // [RW] IO port C reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t gpiodrst : 1; // [RW] IO port D reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t gpioerst : 1; // [RW] IO port E reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd5_6 : 2; // Reserved
        uint32_t gpiohrst : 1; // [RW] IO port H reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd8_11 : 4; // Reserved
        uint32_t crcrst : 1; // [RW] CRC reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd13_20 : 8; // Reserved
        uint32_t dma1rst : 1; // [RW] DMA1 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t dma2rst : 1; // [RW] DMA2 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd23_31 : 9; // Reserved
    }bits;
} rcc_reg_ahb1rstr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t rsvd0_6 : 7; // Reserved
        uint32_t otgfsrst : 1; // [RW] USB OTG FS reset.
        uint32_t rsvd8_31 : 24; // Reserved
    }bits;
} rcc_reg_ahb2rstr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t tim2rst : 1; // [RW] TIM2 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t tim3rst : 1; // [RW] TIM3 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t tim4rst : 1; // [RW] TIM4 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t tim5rst : 1; // [RW] TIM5 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd4_10 : 7; // Reserved
        uint32_t wwdgrst : 1; // [RW] Window Watchdog reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd12_13 : 2; // Reserved
        uint32_t spi2rst : 1; // [RW] SPI2 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t spi3rst : 1; // [RW] SPI3 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd16 : 1; // Reserved
        uint32_t usart2rst : 1; // [RW] USART2 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd18_20 : 3; // Reserved
        uint32_t i2c1rst : 1; // [RW] I2C1 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t i2c2rst : 1; // [RW] I2C2 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t i2c3rst : 1; // [RW] I2C3 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd24_27 : 4; // Reserved
        uint32_t pwrrst : 1; // [RW] Power Interface reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd29_31 : 3; // Reserved
    }bits;
} rcc_reg_apb1rstr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t tim1rst : 1; // [RW] TIM1 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd1_3 : 3; // Reserved
        uint32_t usart1rst : 1; // [RW] USART1 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t usart6rst : 1; // [RW] USART6 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd6_7 : 2; // Reserved
        uint32_t adc1rst : 1; // [RW] ADC1 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd9_10 : 2; // Reserved
        uint32_t sdiorst : 1; // [RW] SDIO reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t spi1rst : 1; // [RW] SPI1 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t spi4rst : 1; // [RW] SPI4 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t syscfgrst : 1; // [RW] System Configuration Controller reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd15 : 1; // Reserved
        uint32_t tim9rst : 1; // [RW] TIM9 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t tim10rst : 1; // [RW] TIM10 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t tim11rst : 1; // [RW] TIM11 reset. Set and cleared by software. '1' - reset, '0' - out of reset.
        uint32_t rsvd19_31 : 13; // Reserved.
    }bits;
} rcc_reg_apb2rstr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t gpioaen : 1; // [RW] IO port A clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t gpioben : 1; // [RW] IO port B clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t gpiocen : 1; // [RW] IO port C clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t gpioden : 1; // [RW] IO port D clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t gpioeen : 1; // [RW] IO port E clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd5_6 : 2; // Reserved
        uint32_t gpiohen : 1; // [RW] IO port H clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd8_11 : 4; // Reserved
        uint32_t crcen : 1; // [RW] CRC clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd13_20 : 8; // Reserved
        uint32_t dma1en : 1; // [RW] DMA1 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t dma2en : 1; // [RW] DMA2 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd23_31 : 9;
    }bits;
} rcc_reg_ahb1enr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t rsvd0_6 : 7; // Reserved
        uint32_t otgfsen : 1; // [RW] USB OTG FS clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd8_31 : 24; // Reserved
    }bits;
} rcc_reg_ahb2enr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t tim2en : 1; // [RW] TIM2 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim3en : 1; // [RW] TIM3 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim4en : 1; // [RW] TIM4 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim5en : 1; // [RW] TIM5 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd4_10 : 7; // Reserved
        uint32_t wwdgen : 1; // [RW] Window Watchdog clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd12_13 : 2; // Reserved
        uint32_t spi2en : 1; // [RW] SPI2 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t spi3en : 1; // [RW] SPI3 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd16 : 1; // Reserved
        uint32_t usart2en : 1; // [RW] USART2 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd18_20 : 3; // Reserved
        uint32_t i2c1en : 1; // [RW] I2C1 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t i2c2en : 1; // [RW] I2C2 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t i2c3en : 1; // [RW] I2C3 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd24_27 : 4; // Reserved
        uint32_t pwren : 1; // [RW] Power Interface clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd29_31 : 3; // Reserved
    }bits;
} rcc_reg_apb1enr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t tim1en : 1; // [RW] TIM1 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd1_3 : 3; // Reserved
        uint32_t usart1en : 1; // [RW] USART1 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t usart6en : 1; // [RW] USART6 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd6_7 : 2; // Reserved
        uint32_t adc1en : 1; // [RW] ADC1 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd9_10 : 2; // Reserved
        uint32_t sdioen : 1; // [RW] SDIO clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t spi1en : 1; // [RW] SPI1 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t spi4en : 1; // [RW] SPI4 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t syscfgen : 1; // [RW] System Configuration Controller clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd15 : 1; // Reserved
        uint32_t tim9en : 1; // [RW] TIM9 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim10en : 1; // [RW] TIM10 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim11en : 1; // [RW] TIM11 clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd19_31 : 13; // Reserved.
    }bits;
} rcc_reg_apb2enr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t gpioalpen : 1; // [RW] IO port A clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t gpioblpen : 1; // [RW] IO port B clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t gpioclpen : 1; // [RW] IO port C clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t gpiodlpen : 1; // [RW] IO port D clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t gpioelpen : 1; // [RW] IO port E clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd5_6 : 2; // Reserved
        uint32_t gpiohlpen : 1; // [RW] IO port H clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd8_11 : 4; // Reserved
        uint32_t crclpen : 1; // [RW] CRC clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd13_14 : 2; // Reserved
        uint32_t flitflpen : 1; // [RW] Flash interface clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t sram1lpen : 1; // [RW] SRAM1 interface clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd17_20 : 4; // Reserved
        uint32_t dma1lpen : 1; // [RW] DMA1 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t dma2lpen : 1; // [RW] DMA2 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd23_31 : 9; // Reserved
    }bits;
} rcc_reg_ahb1lpenr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t rsvd0_6 : 7; // Reserved
        uint32_t otgfslpen : 1; // [RW] USB OTG FS clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd8_31 : 24; // Reserved
    }bits;
} rcc_reg_ahb2lpenr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t tim2lpen : 1; // [RW] TIM2 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim3lpen : 1; // [RW] TIM3 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim4lpen : 1; // [RW] TIM4 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim5lpen : 1; // [RW] TIM5 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd4_10 : 7; // Reserved
        uint32_t wwdglpen : 1; // [RW] Window Watchdog clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd12_13 : 2; // Reserved
        uint32_t spi2lpen : 1; // [RW] SPI2 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t spi3lpen : 1; // [RW] SPI3 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd16 : 1; // Reserved
        uint32_t usart2lpen : 1; // [RW] USART2 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd18_20 : 3; // Reserved
        uint32_t i2c1lpen : 1; // [RW] I2C1 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t i2c2lpen : 1; // [RW] I2C2 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t i2c3lpen : 1; // [RW] I2C3 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd24_27 : 4; // Reserved
        uint32_t pwrlpen : 1; // [RW] Power Interface clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd29_31 : 3; // Reserved
    }bits;
} rcc_reg_apb1lpenr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t tim1lpen : 1; // [RW] TIM1 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd1_3 : 3; // Reserved
        uint32_t usart1lpen : 1; // [RW] USART1 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t usart6lpen : 1; // [RW] USART6 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd6_7 : 2; // Reserved
        uint32_t adc1lpen : 1; // [RW] ADC1 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd9_10 : 2; // Reserved
        uint32_t sdiolpen : 1; // [RW] SDIO clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t spi1lpen : 1; // [RW] SPI1 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t spi4lpen : 1; // [RW] SPI4 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t syscfglpen : 1; // [RW] System Configuration Controller clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd15 : 1; // Reserved
        uint32_t tim9lpen : 1; // [RW] TIM9 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim10lpen : 1; // [RW] TIM10 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t tim11lpen : 1; // [RW] TIM11 clock enable during sleep mode. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t rsvd19_31 : 13; // Reserved.
    }bits;
} rcc_reg_apb2lpenr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t lseon : 1; // [RW] External Low Speed Oscillator Enable. Set and cleared by software. '1' - LSE clock on, '0' - LSE clock off.
        /*
         * Set and cleared by hardware to indicate when the external 32 kHz oscillator is stable. After
        the LSEON bit is cleared, LSERDY goes low after 6 external low-speed oscillator clock
        cycles.
        0: LSE clock not ready
        1: LSE clock ready
         */
        uint32_t lserdy : 1; // [RO] External Low Speed Oscillator Ready.
        /*
         * Set and cleared by software to bypass oscillator in debug mode. This bit can be written only
        when the LSE clock is disabled.
        0: LSE oscillator not bypassed
        1: LSE oscillator bypassed
         */
        uint32_t lsebyp : 1; // [RW] External Low Speed Oscillator Bypass.
        uint32_t rsvd3_7 : 5; // Reserved
        /*
         * Set by software to select the clock source for the RTC. Once the RTC clock source has been
        selected, it cannot be changed anymore unless the Backup domain is reset. The BDRST bit
        can be used to reset them.
        00: No clock
        01: LSE oscillator clock used as the RTC clock
        10: LSI oscillator clock used as the RTC clock
        11: HSE oscillator clock divided by a programmable prescaler (selection through the
        RTCPRE[4:0] bits in the RCC clock configuration register (RCC_CFGR)) used as the RTC
        clock
         */
        uint32_t rtcsel : 2; // [RW] RTC clock source selection.
        uint32_t rsvd10_14 : 5; // Reserved
        uint32_t rtcen : 1; // [RW] RTC clock enable. Set and cleared by software. '1' - clock enabled, '0' - clock disabled.
        uint32_t bdrst : 1; // [RW] Backup domain software reset. Set and cleared by software. '1' - reset, '0' - out of reset..
        uint32_t rsvd17_31 : 15; // Reserved
    }bits;
} rcc_reg_bdcr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t lsion : 1; // [RW] Internal Low Speed Oscillator enable. Set and cleared by software. '1' - enabled, '0' - disabled.
        /*
         * Set and cleared by hardware to indicate when the internal RC 40 kHz oscillator is stable.
        After the LSION bit is cleared, LSIRDY goes low after 3 LSI clock cycles.
        0: LSI RC oscillator not ready
        1: LSI RC oscillator ready
         */
        uint32_t lsirdy : 1; // [RO] Internal Low Speed Oscillator ready.
        uint32_t rsvd2_23 : 22; // Reserved
        uint32_t rmvf : 1; // [RT_W] Remove Reset Flag. Set by software to clear reset flags. '1' - clear reset flags, '0' - no effect.
        /*
         * Cleared by software by writing the RMVF bit.
        Set by hardware when a POR/PDR or BOR reset occurs.
        0: No POR/PDR or BOR reset occurred
        1: POR/PDR or BOR reset occurred
         */
        uint32_t borrstf : 1; // [RO] BOR reset flag.
        /*
         * Set by hardware when a reset from the NRST pin occurs.
        Cleared by writing to the RMVF bit.
        0: No reset from NRST pin occurred
        1: Reset from NRST pin occurred
         */
        uint32_t pinrstf : 1; // [RO] PIN reset flag.
        /*
         * Set by hardware when a POR/PDR reset occurs.
        Cleared by writing to the RMVF bit.
        0: No POR/PDR reset occurred
        1: POR/PDR reset occurred
         */
        uint32_t porrstf : 1; // [RO] POR / PDR reset flag.
        /*
         * Set by hardware when a software reset occurs.
        Cleared by writing to the RMVF bit.
        0: No software reset occurred
        1: Software reset occurred
         */
        uint32_t sftrstf : 1; // [RO] Software reset flag.
        /*
         * Set by hardware when an independent watchdog reset from VDD domain occurs.
        Cleared by writing to the RMVF bit.
        0: No watchdog reset occurred
        1: Watchdog reset occurred
         */
        uint32_t iwdgrstf : 1; // [RO] Independent watchdog reset flag.
        /*
         * Set by hardware when a window watchdog reset occurs.
        Cleared by writing to the RMVF bit.
        0: No window watchdog reset occurred
        1: Window watchdog reset occurred
         */
        uint32_t wwdgrstf : 1; // [RO] Window watchdog reset flag.
        /*
         * Set by hardware when a Low-power management reset occurs.
        Cleared by writing to the RMVF bit.
        0: No Low-power management reset occurred
        1: Low-power management reset occurred
         */
        uint32_t lpwrrstf : 1; // [RO] Low-power reset flag.
    }bits;
} rcc_reg_csr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Set and cleared by software. To write before setting CR[24]=PLLON bit.
        Configuration input for modulation profile period.
         */
        uint32_t modper : 14; // [RW] Modulation Period.
        /*
         * Set and cleared by software. To write before setting CR[24]=PLLON bit.
        Configuration input for modulation profile amplitude.
         */
        uint32_t incstep : 14; // [RW] Incrementation Step.
        uint32_t rsvd28_29 : 2; // Reserved
        /*
         * Set and cleared by software.
        To write before to set CR[24]=PLLON bit.
        0: Center spread
        1: Down spread
         */
        uint32_t spreadsel : 1; // [RW] Spread Select.
        /*
         * Set and cleared by software.
        0: Spread spectrum modulation DISABLE. (To write after clearing CR[24]=PLLON bit)
        1: Spread spectrum modulation ENABLE. (To write before setting CR[24]=PLLON bit)
         */
        uint32_t sscgen : 1; // [RW] Spread Spectrum Modulation Enable.
    }bits;
} rcc_reg_sscfgr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t rsvd0_5 : 6; // Reserved
        /*
         * Set and cleared by software to control the multiplication factor of the VCO. These bits can
        be written only when the PLLI2S is disabled. Only half-word and word accesses are allowed
        to write these bits.
        Caution: The software has to set these bits correctly to ensure that the VCO output
        frequency is between 192 and 432 MHz. With VCO input frequency ranges from 1
        to 2 MHz
        VCO output frequency = VCO input frequency × PLLI2SN with 192 ≤PLLI2SN ≤432
        000000000: PLLI2SN = 0, wrong configuration
        000000001: PLLI2SN = 1, wrong configuration
        ...
        011000000: PLLI2SN = 192
        011000001: PLLI2SN = 193
        011000010: PLLI2SN = 194
        ...
        110110000: PLLI2SN = 432
        110110000: PLLI2SN = 433, wrong configuration
        ...
        111111111: PLLI2SN = 511, wrong configuration
         */
        uint32_t plli2sn : 9; // [RW] PLLI2S multiplication factor for VCO.
        uint32_t rsvd15_27 : 13; // Reserved
        /*
         * Set and cleared by software to control the I2S clock frequency. These bits should be written
        only if the PLLI2S is disabled. The factor must be chosen in accordance with the prescaler
        values inside the I2S peripherals, to reach 0.3% error when using standard crystals and 0%
        error with audio crystals.
        Caution: The I2Ss requires a frequency lower than or equal to 192 MHz to work correctly.
        I2S clock frequency = VCO frequency / PLLR with 2 ≤PLLR ≤7
        000: PLLR = 0, wrong configuration
        001: PLLR = 1, wrong configuration
        010: PLLR = 2
        ...
        111: PLLR = 7
         */
        uint32_t plli2s : 3; // [RW] PLLI2S division factor for I2S clocks.
        uint32_t rsvd31 : 1; // Reserved
    }bits;
} rcc_reg_plli2scfgr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t rsvd0_23 : 24; // Reserved
        /*
         * Set and reset by software to control the clock frequency of all the timers connected to APB1
        and APB2 domain.
        0: If the APB prescaler (PPRE1, PPRE2 in the RCC_CFGR register) is configured to a
        division factor of 1, TIMxCLK = HCKL . Otherwise, the timer clock frequencies are set to
        twice to the frequency of the APB domain to which the timers are connected:
        TIMxCLK = 2xPCLKx.
        1:If the APB prescaler ( PPRE1, PPRE2 in the RCC_CFGR register) is configured to a
        division factor of 1 or 2, TIMxCLK = HCKL. Otherwise, the timer clock frequencies are set to
        four times to the frequency of the APB domain to which the timers are connected:
        TIMxCLK = 4xPCLKx.
         */
        uint32_t timpre : 1; // [RW] Timers clocks prescalers selection.
        uint32_t rsvd25_31 : 7; // Reserved
    }bits;
} rcc_reg_dckcfgr_t;



typedef struct
{
    /*
     * RCC clock control register
       Reset value: 0x0000 XX81 where X is undefined.
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_cr_t RCC_CR;
    /*
     * RCC pll configuration register
     * Reset value: 0x2400 3010
    Access: no wait state, word, half-word and byte access.
    This register is used to configure the PLL clock outputs according to the formulas:
    • f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
    • f(PLL general clock output) = f(VCO clock) / PLLP
    • f(USB OTG FS, SDIO, RNG clock output) = f(VCO clock) / PLLQ
     */
    rcc_reg_pllcfgr_t RCC_PLLCFGR;
    /*
     * RCC clock configuration register
     * Reset value: 0x0000 0000
      Access: 0 ≤ wait state ≤ 2, word, half-word and byte access
      1 or 2 wait states inserted only if the access occurs during a clock source switch
     */
    rcc_reg_cfgr_t RCC_CFGR;
    /*
     * RCC clock interrupt register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_cir_t RCC_CIR;
    /*
     * RCC ahb1 peripheral reset register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_ahb1rstr_t RCC_AHB1RSTR;
    /*
     * RCC ahb2 peripheral reset register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_ahb2rstr_t RCC_AHB2RSTR;
    /*
     * RCC apb1 peripheral reset register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_apb1rstr_t RCC_APB1RSTR;
    /*
     * RCC apb2 peripheral reset register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_apb2rstr_t RCC_APB2RSTR;
    /*
     * RCC ahb1 peripheral clock enable register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_ahb1enr_t RCC_AHB1ENR;
    /*
     * RCC ahb2 peripheral clock enable register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_ahb2enr_t RCC_AHB2ENR;
    /*
     * RCC apb1 peripheral clock enable register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_apb1enr_t RCC_APB1ENR;
    /*
     * RCC apb2 peripheral clock enable register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_apb2enr_t RCC_APB2ENR;
    /*
     * RCC ahb1 peripheral enable in low power mode register
     * Reset value: 0x0061 900F
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_ahb1lpenr_t RCC_AHB1LPENR;
    /*
     * RCC ahb2 peripheral enable in low power mode register
     * Reset value: 0x0000 0080
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_ahb2lpenr_t RCC_AHB2LPENR;
    /*
     * RCC apb1 peripheral enable in low power mode register
     * Reset value: 0x10E2 C80F
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_apb1lpenr_t RCC_APB1LPENR;
    /*
     * RCC apb2 peripheral enable in low power mode register
     * Reset value: 0x0007 7930
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_apb2lpenr_t RCC_APB2LPENR;
    /*
     * RCC Backup domain control register
    Reset value: 0x0000 0000, reset by Backup domain reset.
    Access: 0 ≤ wait state ≤ 3, word, half-word and byte access
    Wait states are inserted in case of successive accesses to this register.
    The LSEON, LSEBYP, RTCSEL and RTCEN bits in the RCC Backup domain control
    register (RCC_BDCR) are in the Backup domain. As a result, after Reset, these bits are
    write-protected and the DBP bit in the PWR power control register (PWR_CR) has to be set
    before these can be modified. These bits are only reset after a Backup domain Reset.
    Any internal or external Reset will not have any effect on these bits.
     */
    rcc_reg_bdcr_t RCC_BDCR;
    /*
     * RCC clock control & status register
    Reset value: 0x0E00 0000, reset by system reset, except reset flags by power reset only.
    Access: 0 ≤ wait state ≤ 3, word, half-word and byte access
    Wait states are inserted in case of successive accesses to this register.
     */
    rcc_reg_csr_t RCC_CSR;
    /*
     * RCC spread spectrum clock generation register
    Reset value: 0x0000 0000
    Access: no wait state, word, half-word and byte access.
    The spread spectrum clock generation is available only for the main PLL.
    The RCC_SSCGR register must be written either before the main PLL is enabled or after
    the main PLL disabled.
     */
    rcc_reg_sscfgr_t RCC_SSCGR;
    /*
     * RCC plli2s configuration register
    Reset value: 0x2400 3000
    Access: no wait state, word, half-word and byte access.
    This register is used to configure the PLLI2S clock outputs according to the formulas:
    • f(VCO clock) = f(PLLI2S clock input) × (PLLI2SN / PLLM)
    • f(PLL I2S clock output) = f(VCO clock) / PLLI2SR
     */
    rcc_reg_plli2scfgr_t RCC_PLLI2SCFGR;
    /*
     * RCC dedicated clocks configuration register
     * Reset value: 0x0000 0000
       Access: no wait state, word, half-word and byte access
     */
    rcc_reg_dckcfgr_t RCC_DCKCFGR;
} rcc_regs_t;
/*
 * ---------- End of RCC registers and bits definitions ----------
 */


/*
 * Since documentation specifies access granularity for each register (i.e. word access, byte access)
 * each access to registers in these functions are performed in a safe manner:
 * - read register as a whole and store its balue to temporary variable
 * - modify the temporary variable
 * - update register as a whole with the temporary variable
 *
 *
 * Theoretically compiler should generate appropriate access when modyfing particular bits,
 * but who knows it if isn't buggy. That's why it's been decided to update register a whole.
 */


static inline volatile rcc_regs_t* rcc_get_regs(void)
{
    return (volatile rcc_regs_t*)(RCC_BASE);
}


void rcc_syscfg_clk_enable(void)
{
    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_apb2enr_t apb2enr = rcc_regs->RCC_APB2ENR;
    apb2enr.bits.syscfgen = 1;
    rcc_regs->RCC_APB2ENR = apb2enr;
    do
    {
        apb2enr = rcc_regs->RCC_APB2ENR;
    }
    while(!(apb2enr.bits.syscfgen));
}


void rcc_syscfg_clk_disable(void)
{
    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_apb2enr_t apb2enr = rcc_regs->RCC_APB2ENR;
    apb2enr.bits.syscfgen = 0;
    rcc_regs->RCC_APB2ENR = apb2enr;
    do
    {
        apb2enr = rcc_regs->RCC_APB2ENR;
    }
    while(apb2enr.bits.syscfgen);
}


void rcc_pwr_clk_enable(void)
{
    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_apb1enr_t apb1enr = rcc_regs->RCC_APB1ENR;
    apb1enr.bits.pwren = 1;
    rcc_regs->RCC_APB1ENR = apb1enr;
    do
    {
        apb1enr = rcc_regs->RCC_APB1ENR;
    }
    while(!(apb1enr.bits.pwren));
}


void rcc_pwr_clk_disable(void)
{
    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_apb1enr_t apb1enr = rcc_regs->RCC_APB1ENR;
    apb1enr.bits.pwren = 0;
    rcc_regs->RCC_APB1ENR = apb1enr;
    do
    {
        apb1enr = rcc_regs->RCC_APB1ENR;
    }
    while(apb1enr.bits.pwren);
}

bool rcc_is_main_pll_on(void)
{
    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_cr_t cr = rcc_regs->RCC_CR;
    if((cr.bits.pllon == 1) && (cr.bits.pllrdy == 1))
    {
        return (bool)(TRUE);
    }
    else
    {
        return (bool)(FALSE);
    }
}

bool rcc_is_hsi_on(void)
{
    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_cr_t cr = rcc_regs->RCC_CR;
    if((cr.bits.hseon == 1) && (cr.bits.hserdy == 1))
    {
        return (bool)(TRUE);
    }
    else
    {
        return (bool)(FALSE);
    }
}

bool rcc_is_hse_on(void)
{
    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_cr_t cr = rcc_regs->RCC_CR;
    if((cr.bits.hsion == 1) && (cr.bits.hsirdy == 1))
    {
        return (bool)(TRUE);
    }
    else
    {
        return (bool)(FALSE);
    }
}

core_ec_t rcc_set_hse_state(rcc_hse_state_t hse_state)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_cr_t cr = rcc_regs->RCC_CR;

    switch(hse_state)
    {
        case RCC_HSE_STATE_DISABLE: // desired: hsebyp = 0, hseon = 0
        {
            cr.bits.hseon = 0;
            rcc_regs->RCC_CR = cr;
            do
            {
                cr = rcc_regs->RCC_CR;
            }while((cr.bits.hseon) && (cr.bits.hserdy));

            cr.bits.hsebyp = 0;
            rcc_regs->RCC_CR = cr;
            do
            {
                cr = rcc_regs->RCC_CR;
            }while(cr.bits.hsebyp);

            break;
        }
        case RCC_HSE_STATE_ENABLE: // desired: hsebyp = 0, hseon = 1
        {
            if(cr.bits.hseon == 1)
            {
                // switch off the hse as hsebyp cannot be altered when it's on
                cr.bits.hseon = 0;
                rcc_regs->RCC_CR = cr;
                do
                {
                    cr = rcc_regs->RCC_CR;
                }while((cr.bits.hseon) && (cr.bits.hserdy));
            }

            cr.bits.hsebyp = 0;
            rcc_regs->RCC_CR = cr;
            do
            {
                cr = rcc_regs->RCC_CR;
            }while(cr.bits.hsebyp);

            cr.bits.hseon = 1;
            rcc_regs->RCC_CR = cr;
            do
            {
                cr = rcc_regs->RCC_CR;
            }while((!(cr.bits.hseon)) && (!(cr.bits.hserdy)));

            break;
        }
        case RCC_HSE_STATE_BYPASS: // desired: hsebyp = 1, hseon = 1
        {
            if(cr.bits.hseon == 1)
            {
                // switch off the hse as hsebyp cannot be altered when it's on
                cr.bits.hseon = 0;
                rcc_regs->RCC_CR = cr;
                do
                {
                    cr = rcc_regs->RCC_CR;
                }while((cr.bits.hseon) && (cr.bits.hserdy));
            }

            cr.bits.hsebyp = 1;
            rcc_regs->RCC_CR = cr;
            do
            {
                cr = rcc_regs->RCC_CR;
            }while(!(cr.bits.hsebyp));

            cr.bits.hseon = 1;
            rcc_regs->RCC_CR = cr;
            do
            {
                cr = rcc_regs->RCC_CR;
            }while((!(cr.bits.hseon)) && (!(cr.bits.hserdy)));

            break;
        }
        default:
        {
            ec = CORE_EC_FAILED;
            break;
        }
    }

    return ec;
}

rcc_sysclk_src_t rcc_get_sysclk_source(void)
{
    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    return (rcc_sysclk_src_t)(rcc_regs->RCC_CFGR.bits.sws);
}

core_ec_t rcc_set_sysclk_source(rcc_sysclk_src_t clk_source)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    switch(clk_source)
    {
        case RCC_SYSCLK_SRC_HSI:
        case RCC_SYSCLK_SRC_HSE:
        case RCC_SYSCLK_SRC_PLL:
        {
            break;
        }
        case RCC_SYSCLK_SRC_RESERVED:
        default:
        {
            ec = CORE_EC_FAILED;
            break;
        }
    }

    if(ec != CORE_EC_FAILED)
    {
        return ec;
    }

    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_cfgr_t cfgr = rcc_regs->RCC_CFGR;
    cfgr.bits.sw = clk_source;
    rcc_regs->RCC_CFGR = cfgr;
    do
    {
        cfgr = rcc_regs->RCC_CFGR;
    }while(cfgr.bits.sws != clk_source);

    return ec;
}

void rcc_main_pll_enable(void)
{
    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_cr_t cr = rcc_regs->RCC_CR;
    cr.bits.pllon = 1;
    rcc_regs->RCC_CR = cr;
    do
    {
        cr = rcc_regs->RCC_CR;
    }
    while((!(cr.bits.pllon)) && (!(cr.bits.pllrdy)));
}

core_ec_t rcc_main_pll_disable(void)
{
    rcc_sysclk_src_t sysclk_src = rcc_get_sysclk_source();
    /*
     * Check if PLL is actually set as SYSCLK main source.
     * PLL cannot be disabled if PLL is currently set as a System Clock (SYSCLK) source.
     * For PLL disablement the SYSCLK should be changed to something other than PLL first.
     * Treat it as error then.
     */
    if((sysclk_src == RCC_SYSCLK_SRC_PLL) || (sysclk_src == RCC_SYSCLK_SRC_RESERVED))
    {
        return CORE_EC_FAILED;
    }

    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_cr_t cr = rcc_regs->RCC_CR;
    cr.bits.pllon = 0;
    rcc_regs->RCC_CR = cr;
    do
    {
        cr = rcc_regs->RCC_CR;
    }
    while((cr.bits.pllon) && (cr.bits.pllrdy));

    return CORE_EC_SUCCESS;
}

core_ec_t rcc_configure_and_enable_pll(
        rcc_pllclk_src_t pllclk_src,
        uint8_t m,
        uint8_t p,
        uint8_t q,
        uint16_t n)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    // check the m correctness
    if((m < 2) || (m > 63))
    {
        return CORE_EC_FAILED;
    }

    // check the p correctness
    switch(p)
    {
        case 2:
        case 4:
        case 6:
        case 8:
        {
            break;
        }
        default:
        {
            ec = CORE_EC_FAILED;
            break;
        }
    }
    if(ec != CORE_EC_SUCCESS)
    {
        return ec;
    }

    // check the q correctness
    if((q < 2) || (q > 15))
    {
        return CORE_EC_FAILED;
    }

    // check the q correctness
    if((n < 50) || (n > 432))
    {
        return CORE_EC_FAILED;
    }

    uint8_t pllsrc = 0;
    switch(pllclk_src)
    {
        case RCC_PLLCLK_SRC_HSE:
        {
            ec = rcc_set_hse_state(RCC_HSE_STATE_ENABLE);
            pllsrc = 1;
            break;
        }
        case RCC_PLLCLK_SRC_HSE_BYPASS:
        {
            ec = rcc_set_hse_state(RCC_HSE_STATE_BYPASS);
            pllsrc = 1;
            break;
        }
        case RCC_PLLCLK_SRC_HSI: // not yet supported by this function, to be implemented in the future
        default:
        {
            ec = CORE_EC_FAILED;
            break;
        }
    }

    if(ec != CORE_EC_SUCCESS)
    {
        return ec;
    }

    ec = rcc_main_pll_disable(); // disable pll for the time of configuration
    if(ec != CORE_EC_SUCCESS)
    {
        return ec;
    }

    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_pllcfgr_t pllcfgr = rcc_regs->RCC_PLLCFGR;
    pllcfgr.bits.pllm = (uint8_t)(m & 0x3F);
    pllcfgr.bits.pllp = (uint8_t)(p & 0x3);
    pllcfgr.bits.pllq = (uint8_t)(q & 0xF);
    pllcfgr.bits.plln = (uint16_t)(n & 0x1FF);
    pllcfgr.bits.pllsrc = (uint8_t)(pllsrc & 0x1);
    rcc_regs->RCC_PLLCFGR = pllcfgr;
    do
    {
        pllcfgr = rcc_regs->RCC_PLLCFGR;
    }
    while((pllcfgr.bits.pllm != m) && (pllcfgr.bits.pllp != p) &&
            (pllcfgr.bits.pllq != q) && (pllcfgr.bits.plln != n) &&
            (pllcfgr.bits.pllsrc != pllsrc));

    rcc_main_pll_enable();

    return ec;
}

core_ec_t rcc_configure_and_enable_main_clocks(
        rcc_sysclk_src_t clk_source,
        rcc_sysclk_to_ahb_div_t sysclk_to_ahb_div,
        rcc_ahb_to_apb1_div_t ahb_to_apb1_div,
        rcc_ahb_to_apb2_div_t ahb_to_apb2_div)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    volatile rcc_regs_t* rcc_regs = rcc_get_regs();
    rcc_reg_cfgr_t cfgr = rcc_regs->RCC_CFGR;
    bool is_clk_source_on = FALSE;

    switch(clk_source)
    {
        case RCC_SYSCLK_SRC_HSI:
        {
            is_clk_source_on = rcc_is_hsi_on();
            break;
        }
        case RCC_SYSCLK_SRC_HSE:
        {
            is_clk_source_on = rcc_is_hse_on();
            break;
        }
        case RCC_SYSCLK_SRC_PLL:
        {
            is_clk_source_on = rcc_is_main_pll_on();
            break;
        }
        case RCC_SYSCLK_SRC_RESERVED:
        default:
        {
            is_clk_source_on = FALSE;
            break;
        }
    }

    if(!is_clk_source_on)
    {
        return CORE_EC_FAILED;
    }

    /*
     * set apb1 and apb2 dividers to maximum division
     * (thats just good manner to low these clks down for the time of AHB/HCLK configuration)
     */
    cfgr.bits.ppre1 = RCC_AHB_TO_APB1_DIV_16;
    cfgr.bits.ppre2 = RCC_AHB_TO_APB2_DIV_16;
    rcc_regs->RCC_CFGR = cfgr;
    do
    {
        cfgr = rcc_regs->RCC_CFGR;
    }while((cfgr.bits.ppre1 != RCC_AHB_TO_APB1_DIV_16) && (cfgr.bits.ppre2 != RCC_AHB_TO_APB2_DIV_16));

    // set the AHB / HCLK desired configuration
    cfgr.bits.hpre = sysclk_to_ahb_div;
    rcc_regs->RCC_CFGR = cfgr;
    do
    {
        cfgr = rcc_regs->RCC_CFGR;
    }while(cfgr.bits.hpre != sysclk_to_ahb_div);

    // switch sysclk to the given clk_source (its all running now!)
    ec = rcc_set_sysclk_source(clk_source);
    if(ec != CORE_EC_SUCCESS)
    {
        return CORE_EC_FAILED;
    }

    // set apb1 and apb2 dividers to desired divisions
    cfgr.bits.ppre1 = ahb_to_apb1_div;
    cfgr.bits.ppre2 = ahb_to_apb2_div;
    rcc_regs->RCC_CFGR = cfgr;
    do
    {
        cfgr = rcc_regs->RCC_CFGR;
    }while((cfgr.bits.ppre1 != ahb_to_apb1_div) && (cfgr.bits.ppre2 != ahb_to_apb2_div));

    return ec;
}
