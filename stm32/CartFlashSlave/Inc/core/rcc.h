/*
 * rcc.h
 *
 *  Created on: 18 Oct 2021
 *      Author: alf64
 *
 * Provides functionality for Reset and Clock Control (RCC) interface.
 */

#ifndef CORE_RCC_H_
#define CORE_RCC_H_

#include <core/ec.h>

#ifdef __cplusplus
 extern "C" {
#endif

/*
 * Possible states for High Speed External oscillator (HSE).
 */
typedef enum
{
    //!< enables the High Speed External oscillator. Means the external crystal is connected to OSC pins.
    RCC_HSE_STATE_ENABLE = 0,
    //!< bypasses the High Speed External oscillator. Means the external clock source is connected to OSC_IN pin.
    RCC_HSE_STATE_BYPASS = 1,
    //<! disables the Hish Speed External oscillator input totally.
    RCC_HSE_STATE_DISABLE = 2
}rcc_hse_state_t;

/*
 * Possible PLL (and PLLI2S) clock sources.
 * (HSE or HSI)
 */
typedef enum
{
    RCC_PLLCLK_SRC_HSI = 0, // HSI as PLL clk source
    RCC_PLLCLK_SRC_HSE = 1, // HSE (oscillator mode) as PLL clk source
    RCC_PLLCLK_SRC_HSE_BYPASS = 2 // HSE (external clk mode) as PLL clk source
}rcc_pllclk_src_t;

/*
 * Possible System Clock (SYSCLK) sources.
 * (HSE, HSI or PLL)
 */
typedef enum
{
    RCC_SYSCLK_SRC_HSI = 0,
    RCC_SYSCLK_SRC_HSE = 1,
    RCC_SYSCLK_SRC_PLL = 2,
    RCC_SYSCLK_SRC_RESERVED = 3 // reserved value, should not be used
}rcc_sysclk_src_t;

/*
 * Possible divider (clock division factor) values for AHB.
 * The clock being divided is SYSCLK. The resulting clock after division is AHB.
 */
typedef enum
{
    RCC_SYSCLK_TO_AHB_DIV_1 = 0,
    RCC_SYSCLK_TO_AHB_DIV_2 = 8,
    RCC_SYSCLK_TO_AHB_DIV_4 = 9,
    RCC_SYSCLK_TO_AHB_DIV_8 = 10,
    RCC_SYSCLK_TO_AHB_DIV_16 = 11,
    RCC_SYSCLK_TO_AHB_DIV_64 = 12,
    RCC_SYSCLK_TO_AHB_DIV_128 = 13,
    RCC_SYSCLK_TO_AHB_DIV_256 = 14,
    RCC_SYSCLK_TO_AHB_DIV_512 = 15
}rcc_sysclk_to_ahb_div_t;

/*
 * Possible divider (clock division factor) values for APB1.
 * The clock being divided is AHB. The resulting clock after division is APB1 (APB low-speed).
 */
typedef enum
{
    RCC_AHB_TO_APB1_DIV_1 = 0,
    RCC_AHB_TO_APB1_DIV_2 = 4,
    RCC_AHB_TO_APB1_DIV_4 = 5,
    RCC_AHB_TO_APB1_DIV_8 = 6,
    RCC_AHB_TO_APB1_DIV_16 = 7
}rcc_ahb_to_apb1_div_t;

/*
 * Possible divider (clock division factor) values for APB2.
 * The clock being divided is AHB. The resulting clock after division is APB2 (APB high-speed).
 */
typedef enum
{
    RCC_AHB_TO_APB2_DIV_1 = 0,
    RCC_AHB_TO_APB2_DIV_2 = 4,
    RCC_AHB_TO_APB2_DIV_4 = 5,
    RCC_AHB_TO_APB2_DIV_8 = 6,
    RCC_AHB_TO_APB2_DIV_16 = 7
}rcc_ahb_to_apb2_div_t;

/*
 * Enables clock for System Configuration Controller (SYSCFG).
 * This is needed to use SYSCFG.
 * SYSCFG is mainly used to remap the memory accessible in the code area and
 * manage the GPIO external interrupts.
 */
void rcc_syscfg_clk_enable(void);

//!< Disables clock for System Configuration Controller (SYSCFG).
void rcc_syscfg_clk_disable(void);

//!< Enables clock for Power Controller
void rcc_pwr_clk_enable(void);

//!< Disables clock for Power Controller
void rcc_pwr_clk_disable(void);

/*
 * @brief Sets the High Speed External oscillator (HSE) to given state.
 * @param hse_state A state to be set.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If unable to set the hse state.
 * @retval CORE_EC_SUCCESS If succeeded to set the hse state.
 */
core_ec_t rcc_set_hse_state(rcc_hse_state_t hse_state);

/*
 * @brief Gets the current System Clock Source (SYSCLK).
 * @returns rcc_sysclk_scr_t
 */
rcc_sysclk_src_t rcc_get_sysclk_source(void);

/*
 * @brief Sets the current System Clock Source (SYSCLK).
 *
 * @note
 * Make sure you configure and enable the clock source first,
 * before setting it as system clock source.
 *
 * @attention
 * This function does not enable either configure the clk_source, you
 * have to make sure it is configured and enabled first.
 *
 * @returns core_ec_t An error code.
 * @retval CORE_EC_FAILED If failed to set the system clock source..
 * @retval CORE_EC_SUCCESS If succeeded to set the system clock source.
 */
core_ec_t rcc_set_sysclk_source(rcc_sysclk_src_t clk_source);

//!< Returns TRUE if main PLL is ON, otherwirse returns FALSE.
bool rcc_is_main_pll_on(void);

//!< Returns TRUE if HSI is ON, otherwirse returns FALSE.
bool rcc_is_hsi_on(void);

//!< Returns TRUE if HSE is ON, otherwirse returns FALSE.
bool rcc_is_hse_on(void);

/*
 * @brief Enables the main PLL.
 */
void rcc_main_pll_enable(void);

/*
 * @brief Disables the main PLL.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If unable to disable the main PLL.
 * @retval CORE_EC_SUCCESS If succeeded to disable the main PLL.
 */
core_ec_t rcc_main_pll_disable(void);

/*
 * @brief Configures and enables (main) PLL.
 *
 * @details
 * This function configures the main PLL, enables the PLL clock source and
 * enables PLL.
 *
 * @attention
 * pllclk_src parameter sets the clock source for PLL. This can be HSE or HSI.
 * This function automatically enables such clock source so there is no need to do it separately.
 * I.e.: if RCC_PLLCLK_SRC_HSE (or RCC_PLLCLK_SRC_HSE_BYPASS) is chosen as pllclk_src,
 * HSE will be enabled via rcc_set_hse_state() call from within this function.
 * PLL is enabled via rcc_main_pll_enable() called from within this function.
 *
 * @param pllclk_src A clock source for the PLL.
 * @param m One of PLL division factor. Refer to uC reference manual -> clock tree for the details.
            Specifies the division factor for PLL VCO input clock
            This parameter must be a number between Min_Data = 2 and Max_Data = 63.
            You have to set the m parameter correctly to ensure that the VCO input
            frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
            of 2 MHz to limit PLL jitter.
 * @param p One of PLL division factor. Refer to uC reference manual -> clock tree for the details.
 *          Specifies the division factor for main system clock (SYSCLK)
 *          This parameter must be a number in the range {2, 4, 6, or 8}.
 * @param q One of PLL division factor. Refer to uC reference manual -> clock tree for the details.
 *          Specifies the division factor for OTG FS, SDIO and RNG clocks.
 *          This parameter must be a number between Min_Data = 2 and Max_Data = 15.
 *          If the USB OTG FS is used in your application, you have to set the
 *          q parameter correctly to have 48 MHz clock for the USB. However,
 *          the SDIO and RNG need a frequency lower than or equal to 48 MHz to work
 *          correctly.
 * @param n PLL multiplication factor. Refer to uC reference manual -> clock tree for the details.
 *          Specifies the multiplication factor for PLL VCO output clock.
 *          This parameter must be a number between Min_Data = 50 and Max_Data = 432
 *          Except for STM32F411xE devices where Min_Data = 192.
 *  @note   You have to set the n parameter correctly to ensure that the VCO
 *          output frequency is between 100 and 432 MHz, Except for STM32F411xE devices
 *          where frequency is between 192 and 432 MHz.
 *
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If unable to configure & enable pll.
 * @retval CORE_EC_SUCCESS If succeeded to configure & enable pll.
 */
core_ec_t rcc_configure_and_enable_pll(
        rcc_pllclk_src_t pllclk_src,
        uint8_t m,
        uint8_t p,
        uint8_t q,
        uint16_t n);

/*
 * @brief Configures main system clocks (for HCLK / AHB, APB1, APB2).
 *
 * @attention
 * Remember to set flash latency according to desired clock settings.
 * @see flash_set_latency()
 *
 * @attention
 * This function sets the system clock source (SYSCLK) by calling the
 * rcc_set_sysclk_source(clk_source), so you don't have to do it
 * on your own. Just make sure the clk_source is enabled.
 *
 * @param sysclk_to_ahb_div     Prescaler value for AHB.
 * @param ahb_to_apb1_div       Prescaler value for APB1.
 * @param ahb_to_apb2_div       Prescaler value for APB2.
 *
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If unable to configure and enable main clocks.
 * @retval CORE_EC_SUCCESS If succeeded to configure and enable main clocks.
 */
core_ec_t rcc_configure_and_enable_main_clocks(
        rcc_sysclk_src_t clk_source,
        rcc_sysclk_to_ahb_div_t sysclk_to_ahb_div,
        rcc_ahb_to_apb1_div_t ahb_to_apb1_div,
        rcc_ahb_to_apb2_div_t ahb_to_apb2_div);

#ifdef __cplusplus
}
#endif

#endif /* CORE_RCC_H_ */
