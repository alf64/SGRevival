/*
 * gpio.h
 *
 *  Created on: 4 Jan 2022
 *      Author: alf64
 *
 *  Provides functionality for General Purpose I/O (GPIO)
 */

#ifndef CORE_GPIO_H_
#define CORE_GPIO_H_

#include <core/ec.h>

#ifdef __cplusplus
 extern "C" {
#endif

//!< Possible GPIO ports.
typedef enum
{
 GPIO_PORT_A = 0,
 GPIO_PORT_B = 1,
 GPIO_PORT_C = 2,
 GPIO_PORT_D = 3,
 GPIO_PORT_E = 4,
 GPIO_PORT_H = 5
}gpio_port_t;

/*
 * Structure describing the gpio pins, used by the functions below.
 * Setting the field to 1 means the user is interested
 * to alter such pin configuration.
 * Setting the field to 0 means no change shall be made
 * for the pin.
 */
typedef union
{
    uint16_t value;
    struct
    {
        uint16_t pin0 : 1;
        uint16_t pin1 : 1;
        uint16_t pin2 : 1;
        uint16_t pin3 : 1;
        uint16_t pin4 : 1;
        uint16_t pin5 : 1;
        uint16_t pin6 : 1;
        uint16_t pin7 : 1;
        uint16_t pin8 : 1;
        uint16_t pin9 : 1;
        uint16_t pin10 : 1;
        uint16_t pin11 : 1;
        uint16_t pin12 : 1;
        uint16_t pin13 : 1;
        uint16_t pin14 : 1;
        uint16_t pin15 : 1;
    }pins;
}gpio_pins_t;

//!< Possible pin modes.
typedef enum
{
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_ALTERNATE_FUNCTION = 2,
    GPIO_MODE_ANALOG_MODE = 3
}gpio_mode_t;

//!< Possible pin output types.
typedef enum
{
    GPIO_OTYPE_PUSH_PULL = 0,
    GPIO_OTYPE_OPEN_DRAIN = 1
}gpio_otype_t;

/*
 * Possible pin output speeds.
 * Refer to device datasheet for the details.
 */
typedef enum
{
    /*
     * Maximum possible frequencies have been put in the comments.
     * They come from device datasheet.
     * Please note that the maximum frequency is limited by the GPIO clock.
     * GPIO module is being clocked from AHB1 bus.
     */
    // LOW: for Vdd >= 2,7V; Fmax = 4 MHz (for Cl=50pF); Fmax = 8 Mhz (for Cl=10pF)
    GPIO_OSPEED_LOW = 0,
    // MEDIUM: for Vdd >= 2,7V; Fmax = 25 MHz (for Cl=50pF); Fmax = 50 Mhz (for Cl=10pF)
    GPIO_OSPEED_MEDIUM = 1,
    // HIGH: for Vdd >= 2,7V; Fmax = 50 MHz (for Cl=40pF); Fmax = 100 Mhz (for Cl=10pF)
    GPIO_OSPEED_HIGH = 2,
    // VERY HIGH: for Vdd >= 2,7V; Fmax = 100 MHz (for Cl=30pF); Fmax = 180 Mhz (for Cl=10pF)
    GPIO_OSPEED_VERY_HIGH = 3
}gpio_ospeed_t;

//!< Possible pin pull types.
typedef enum
{
    GPIO_PTYPE_NO_PULL = 0,
    GPIO_PTYPE_PULL_UP = 1,
    GPIO_PTYPE_PULL_DOWN = 2,
    GPIO_PTYPE_RESERVED = 3
}gpio_ptype_t;

/* -------------
 * Pins alternative functions for
 * STM32F401xB and STM32F401xC devices.
 * @see datasheet
 * ----------------- */
typedef enum
{
    GPIOA_AF_PIN0_TIM2_CH1_TIM2_ETR = 1,
    GPIOA_AF_PIN0_TIM5_CH1 = 2,
    GPIOA_AF_PIN0_USART2_CTS = 7,
    GPIOA_AF_PIN0_FPU_EVENTOUT = 15
}gpioa_af_pin0_t;

typedef enum
{
    GPIOA_AF_PIN1_TIM2_CH2 = 1,
    GPIOA_AF_PIN1_TIM5_CH2 = 2,
    GPIOA_AF_PIN1_USART2_RTS = 7,
    GPIOA_AF_PIN1_FPU_EVENTOUT = 15
}gpioa_af_pin1_t;

typedef enum
{
    GPIOA_AF_PIN2_TIM2_CH3 = 1,
    GPIOA_AF_PIN2_TIM5_CH3 = 2,
    GPIOA_AF_PIN2_TIM9_CH1 = 3,
    GPIOA_AF_PIN2_USART2_TX = 7,
    GPIOA_AF_PIN2_FPU_EVENTOUT = 15
}gpioa_af_pin2_t;

typedef enum
{
    GPIOA_AF_PIN3_TIM2_CH4 = 1,
    GPIOA_AF_PIN3_TIM5_CH4 = 2,
    GPIOA_AF_PIN3_TIM9_CH2 = 3,
    GPIOA_AF_PIN3_USART2_RX = 7,
    GPIOA_AF_PIN3_FPU_EVENTOUT = 15
}gpioa_af_pin3_t;

typedef enum
{
    GPIOA_AF_PIN4_SPI1_NSS = 5,
    GPIOA_AF_PIN4_SPI3_NSS_I2S3_WS = 6,
    GPIOA_AF_PIN4_USART2_CK = 7,
    GPIOA_AF_PIN4_FPU_EVENTOUT = 15
}gpioa_af_pin4_t;

typedef enum
{
    GPIOA_AF_PIN5_TIM2_CH1_TIM2_ETR = 1,
    GPIOA_AF_PIN5_SPI1_SCK = 5,
    GPIOA_AF_PIN5_FPU_EVENTOUT = 15
}gpioa_af_pin5_t;

typedef enum
{
    GPIOA_AF_PIN6_TIM1_BKIN = 1,
    GPIOA_AF_PIN6_TIM3_CH1 = 2,
    GPIOA_AF_PIN6_SPI1_MISO = 5,
    GPIOA_AF_PIN6_FPU_EVENTOUT = 15
}gpioa_af_pin6_t;

typedef enum
{
    GPIOA_AF_PIN7_TIM1_CH1N = 1,
    GPIOA_AF_PIN7_TIM3_CH2 = 2,
    GPIOA_AF_PIN7_SPI1_MOSI = 5,
    GPIOA_AF_PIN7_FPU_EVENTOUT = 15
}gpioa_af_pin7_t;

typedef enum
{
    GPIOA_AF_PIN8_MCO_1 = 0,
    GPIOA_AF_PIN8_TIM1_CH1 = 1,
    GPIOA_AF_PIN8_I2C3_SCL = 4,
    GPIOA_AF_PIN8_USART1_CK = 7,
    GPIOA_AF_PIN8_OTG_FS_SOF = 10,
    GPIOA_AF_PIN8_FPU_EVENTOUT = 15
}gpioa_af_pin8_t;

typedef enum
{
    GPIOA_AF_PIN9_TIM1_CH2 = 1,
    GPIOA_AF_PIN9_I2C3_SMBA = 4,
    GPIOA_AF_PIN9_USART1_TX = 7,
    GPIOA_AF_PIN9_OTG_FS_VBUS = 10,
    GPIOA_AF_PIN9_FPU_EVENTOUT = 15
}gpioa_af_pin9_t;

typedef enum
{
    GPIOA_AF_PIN10_TIM1_CH3 = 1,
    GPIOA_AF_PIN10_USART1_RX = 7,
    GPIOA_AF_PIN10_OTG_FS_ID = 10,
    GPIOA_AF_PIN10_FPU_EVENTOUT = 15
}gpioa_af_pin10_t;

typedef enum
{
    GPIOA_AF_PIN11_TIM1_CH4 = 1,
    GPIOA_AF_PIN11_USART1_CTS = 7,
    GPIOA_AF_PIN11_USART6_TX = 8,
    GPIOA_AF_PIN11_OTG_FS_DM = 10,
    GPIOA_AF_PIN11_FPU_EVENTOUT = 15
}gpioa_af_pin11_t;

typedef enum
{
    GPIOA_AF_PIN12_TIM1_ETR = 1,
    GPIOA_AF_PIN12_USART1_RTS = 7,
    GPIOA_AF_PIN12_USART6_RX = 8,
    GPIOA_AF_PIN12_OTG_FS_DP = 10,
    GPIOA_AF_PIN12_FPU_EVENTOUT = 15
}gpioa_af_pin12_t;

typedef enum
{
    GPIOA_AF_PIN13_JTMS_SWDIO = 0,
    GPIOA_AF_PIN13_FPU_EVENTOUT = 15
}gpioa_af_pin13_t;

typedef enum
{
    GPIOA_AF_PIN14_JTCK_SWCLK = 0,
    GPIOA_AF_PIN14_FPU_EVENTOUT = 15
}gpioa_af_pin14_t;

typedef enum
{
    GPIOA_AF_PIN15_JTDI = 0,
    GPIOA_AF_PIN15_TIM2_CH1_TIM2_ETR = 1,
    GPIOA_AF_PIN15_SPI1_NSS = 5,
    GPIOA_AF_PIN15_SPI3_NSS_I2S3_WS = 6,
    GPIOA_AF_PIN15_FPU_EVENTOUT = 15
}gpioa_af_pin15_t;

typedef enum
{
    GPIOB_AF_PIN0_TIM1_CH2N = 1,
    GPIOB_AF_PIN0_TIM3_CH3 = 2,
    GPIOB_AF_PIN0_FPU_EVENTOUT = 15
}gpiob_af_pin0_t;

typedef enum
{
    GPIOB_AF_PIN1_TIM1_CH3N = 1,
    GPIOB_AF_PIN1_TIM3_CH4 = 2,
    GPIOB_AF_PIN1_FPU_EVENTOUT = 15
}gpiob_af_pin1_t;

typedef enum
{
    GPIOB_AF_PIN2_FPU_EVENTOUT = 15
}gpiob_af_pin2_t;

typedef enum
{
    GPIOB_AF_PIN3_JTDO_SWO = 0,
    GPIOB_AF_PIN3_TIM2_CH2 = 1,
    GPIOB_AF_PIN3_SPI1_SCK = 5,
    GPIOB_AF_PIN3_SPI3_SCK_I2S3_CK = 6,
    GPIOB_AF_PIN3_I2C2_SDA = 9,
    GPIOB_AF_PIN3_FPU_EVENTOUT = 15
}gpiob_af_pin3_t;

typedef enum
{
    GPIOB_AF_PIN4_JTRST = 0,
    GPIOB_AF_PIN4_TIM3_CH1 = 2,
    GPIOB_AF_PIN4_SPI1_MISO = 5,
    GPIOB_AF_PIN4_SPI3_MISO = 6,
    GPIOB_AF_PIN4_I2S3ext_SD = 7,
    GPIOB_AF_PIN4_I2C3_SDA = 9,
    GPIOB_AF_PIN4_FPU_EVENTOUT = 15
}gpiob_af_pin4_t;

typedef enum
{
    GPIOB_AF_PIN5_TIM3_CH2 = 2,
    GPIOB_AF_PIN5_I2C1_SMBA = 4,
    GPIOB_AF_PIN5_SPI1_MOSI = 5,
    GPIOB_AF_PIN5_SPI3_MOSI_I2S3_SD = 6,
    GPIOB_AF_PIN5_FPU_EVENTOUT = 15
}gpiob_af_pin5_t;

typedef enum
{
    GPIOB_AF_PIN6_TIM4_CH1 = 2,
    GPIOB_AF_PIN6_I2C1_SCL = 4,
    GPIOB_AF_PIN6_USART1_TX = 7,
    GPIOB_AF_PIN6_FPU_EVENTOUT = 15
}gpiob_af_pin6_t;

typedef enum
{
    GPIOB_AF_PIN7_TIM4_CH2 = 2,
    GPIOB_AF_PIN7_I2C1_SDA = 4,
    GPIOB_AF_PIN7_USART1_RX = 7,
    GPIOB_AF_PIN7_FPU_EVENTOUT = 15
}gpiob_af_pin7_t;

typedef enum
{
    GPIOB_AF_PIN8_TIM4_CH3 = 2,
    GPIOB_AF_PIN8_TIM10_CH1 = 3,
    GPIOB_AF_PIN8_I2C1_SCL = 4,
    GPIOB_AF_PIN8_SDIO_D4 = 12,
    GPIOB_AF_PIN8_FPU_EVENTOUT = 15
}gpiob_af_pin8_t;

typedef enum
{
    GPIOB_AF_PIN9_TIM4_CH4 = 2,
    GPIOB_AF_PIN9_TIM11_CH1 = 3,
    GPIOB_AF_PIN9_I2C1_SDA = 4,
    GPIOB_AF_PIN9_SPI2_NSS_I2S2_WS = 5,
    GPIOB_AF_PIN9_SDIO_D5 = 12,
    GPIOB_AF_PIN9_FPU_EVENTOUT = 15
}gpiob_af_pin9_t;

typedef enum
{
    GPIOB_AF_PIN10_TIM2_CH3 = 1,
    GPIOB_AF_PIN10_I2C2_SCL = 4,
    GPIOB_AF_PIN10_SPI2_SCK_I2S2_CK = 5,
    GPIOB_AF_PIN10_FPU_EVENTOUT = 15
}gpiob_af_pin10_t;

typedef enum
{
    GPIOB_AF_PIN11_TIM2_CH4 = 1,
    GPIOB_AF_PIN11_I2C2_SDA = 4,
    GPIOB_AF_PIN11_FPU_EVENTOUT = 15
}gpiob_af_pin11_t;

typedef enum
{
    GPIOB_AF_PIN12_TIM1_BKIN = 1,
    GPIOB_AF_PIN12_I2C2_SMBA = 4,
    GPIOB_AF_PIN12_SPI2_NSS_I2S2_WS = 5,
    GPIOB_AF_PIN12_FPU_EVENTOUT = 15
}gpiob_af_pin12_t;

typedef enum
{
    GPIOB_AF_PIN13_TIM1_CH1N = 1,
    GPIOB_AF_PIN13_SPI2_SCK_I2S2_CK = 5,
    GPIOB_AF_PIN13_FPU_EVENTOUT = 15
}gpiob_af_pin13_t;

typedef enum
{
    GPIOB_AF_PIN14_TIM1_CH2N = 1,
    GPIOB_AF_PIN14_SPI2_MISO = 5,
    GPIOB_AF_PIN14_I2S2ext_SD = 6,
    GPIOB_AF_PIN14_FPU_EVENTOUT = 15
}gpiob_af_pin14_t;

typedef enum
{
    GPIOB_AF_PIN15_RTC_REFN = 0,
    GPIOB_AF_PIN15_TIM1_CH3N = 1,
    GPIOB_AF_PIN15_SPI2_MOSI_I2S2_SD = 5,
    GPIOB_AF_PIN15_FPU_EVENTOUT = 15
}gpiob_af_pin15_t;

typedef enum
{
    GPIOC_AF_PIN0_FPU_EVENTOUT = 15
}gpioc_af_pin0_t;

typedef enum
{
    GPIOC_AF_PIN1_FPU_EVENTOUT = 15
}gpioc_af_pin1_t;

typedef enum
{
    GPIOC_AF_PIN2_SPI2_MISO = 5,
    GPIOC_AF_PIN2_I2S2ext_SD = 6,
    GPIOC_AF_PIN2_FPU_EVENTOUT = 15
}gpioc_af_pin2_t;

typedef enum
{
    GPIOC_AF_PIN3_SPI2_MOSI_I2S2_SD = 5,
    GPIOC_AF_PIN3_FPU_EVENTOUT = 15
}gpioc_af_pin3_t;

typedef enum
{

    GPIOC_AF_PIN4_FPU_EVENTOUT = 15
}gpioc_af_pin4_t;

typedef enum
{
    GPIOC_AF_PIN5_FPU_EVENTOUT = 15
}gpioc_af_pin5_t;

typedef enum
{
    GPIOC_AF_PIN6_TIM3_CH1 = 2,
    GPIOC_AF_PIN6_I2S2_MCK = 5,
    GPIOC_AF_PIN6_USART6_TX = 8,
    GPIOC_AF_PIN6_SDIO_D6 = 12,
    GPIOC_AF_PIN6_FPU_EVENTOUT = 15
}gpioc_af_pin6_t;

typedef enum
{
    GPIOC_AF_PIN7_TIM3_CH2 = 2,
    GPIOC_AF_PIN7_I2S3_MCK = 6,
    GPIOC_AF_PIN7_USART6_RX = 8,
    GPIOC_AF_PIN7_SDIO_D7 = 12,
    GPIOC_AF_PIN7_FPU_EVENTOUT = 15
}gpioc_af_pin7_t;

typedef enum
{
    GPIOC_AF_PIN8_TIM3_CH3 = 2,
    GPIOC_AF_PIN8_USART6_CK = 8,
    GPIOC_AF_PIN8_SDIO_D0 = 12,
    GPIOC_AF_PIN8_FPU_EVENTOUT = 15
}gpioc_af_pin8_t;

typedef enum
{
    GPIOC_AF_PIN9_MCO_2 = 0,
    GPIOC_AF_PIN9_TIM3_CH4 = 2,
    GPIOC_AF_PIN9_I2C3_SDA = 4,
    GPIOC_AF_PIN9_I2S2_CKIN = 5,
    GPIOC_AF_PIN9_SDIO_D1 = 12,
    GPIOC_AF_PIN9_FPU_EVENTOUT = 15
}gpioc_af_pin9_t;

typedef enum
{
    GPIOC_AF_PIN10_SPI3_SCK_I2S3_CK = 6,
    GPIOC_AF_PIN10_SDIO_D2 = 12,
    GPIOC_AF_PIN10_FPU_EVENTOUT = 15
}gpioc_af_pin10_t;

typedef enum
{
    GPIOC_AF_PIN11_I2S3ext_SD = 5,
    GPIOC_AF_PIN11_SPI3_MISO = 6,
    GPIOC_AF_PIN11_SDIO_D3 = 12,
    GPIOC_AF_PIN11_FPU_EVENTOUT = 15
}gpioc_af_pin11_t;

typedef enum
{
    GPIOC_AF_PIN12_SPI3_MOSI_I2S3_SD = 6,
    GPIOC_AF_PIN12_SDIO_CK = 12,
    GPIOC_AF_PIN12_FPU_EVENTOUT = 15
}gpioc_af_pin12_t;

typedef enum
{
    GPIOC_AF_PIN13_FPU_EVENTOUT = 15
}gpioc_af_pin13_t;

typedef enum
{
    GPIOC_AF_PIN14_FPU_EVENTOUT = 15
}gpioc_af_pin14_t;

typedef enum
{
    GPIOC_AF_PIN15_FPU_EVENTOUT = 15
}gpioc_af_pin15_t;

typedef enum
{
    GPIOD_AF_PIN0_FPU_EVENTOUT = 15
}gpiod_af_pin0_t;

typedef enum
{
    GPIOD_AF_PIN1_FPU_EVENTOUT = 15
}gpiod_af_pin1_t;

typedef enum
{
    GPIOD_AF_PIN2_TIM3_ETR = 2,
    GPIOD_AF_PIN2_SDIO_CMD = 12,
    GPIOD_AF_PIN2_FPU_EVENTOUT = 15
}gpiod_af_pin2_t;

typedef enum
{
    GPIOD_AF_PIN3_SPI2_SCK_I2S2_CK = 5,
    GPIOD_AF_PIN3_USART2_CTS = 7,
    GPIOD_AF_PIN3_FPU_EVENTOUT = 15
}gpiod_af_pin3_t;

typedef enum
{
    GPIOD_AF_PIN4_USART2_RTS = 7,
    GPIOD_AF_PIN4_FPU_EVENTOUT = 15
}gpiod_af_pin4_t;

typedef enum
{
    GPIOD_AF_PIN5_USART2_TX = 7,
    GPIOD_AF_PIN5_FPU_EVENTOUT = 15
}gpiod_af_pin5_t;

typedef enum
{
    GPIOD_AF_PIN6_SPI3_MOSI_I2S3_SD = 5,
    GPIOD_AF_PIN6_USART2_RX = 7,
    GPIOD_AF_PIN6_FPU_EVENTOUT = 15
}gpiod_af_pin6_t;

typedef enum
{
    GPIOD_AF_PIN7_USART2_CK = 7,
    GPIOD_AF_PIN7_FPU_EVENTOUT = 15
}gpiod_af_pin7_t;

typedef enum
{
    GPIOD_AF_PIN8_FPU_EVENTOUT = 15
}gpiod_af_pin8_t;

typedef enum
{
    GPIOD_AF_PIN9_FPU_EVENTOUT = 15
}gpiod_af_pin9_t;

typedef enum
{
    GPIOD_AF_PIN10_FPU_EVENTOUT = 15
}gpiod_af_pin10_t;

typedef enum
{
    GPIOD_AF_PIN11_FPU_EVENTOUT = 15
}gpiod_af_pin11_t;

typedef enum
{
    GPIOD_AF_PIN12_TIM4_CH1 = 2,
    GPIOD_AF_PIN12_FPU_EVENTOUT = 15
}gpiod_af_pin12_t;

typedef enum
{
    GPIOD_AF_PIN13_TIM4_CH2 = 2,
    GPIOD_AF_PIN13_FPU_EVENTOUT = 15
}gpiod_af_pin13_t;

typedef enum
{
    GPIOD_AF_PIN14_TIM4_CH3 = 2,
    GPIOD_AF_PIN14_FPU_EVENTOUT = 15
}gpiod_af_pin14_t;

typedef enum
{
    GPIOD_AF_PIN15_TIM4_CH4 = 2,
    GPIOD_AF_PIN15_FPU_EVENTOUT = 15
}gpiod_af_pin15_t;

typedef enum
{
    GPIOE_AF_PIN0_TIM4_ETR = 2,
    GPIOE_AF_PIN0_FPU_EVENTOUT = 15
}gpioe_af_pin0_t;

typedef enum
{
    GPIOE_AF_PIN1_TIM1_CH2N = 1,
    GPIOE_AF_PIN1_FPU_EVENTOUT = 15
}gpioe_af_pin1_t;

typedef enum
{
    GPIOE_AF_PIN2_TRACECLK = 0,
    GPIOE_AF_PIN2_SPI4_SCK = 5,
    GPIOE_AF_PIN2_FPU_EVENTOUT = 15
}gpioe_af_pin2_t;

typedef enum
{
    GPIOE_AF_PIN3_TRACED0 = 0,
    GPIOE_AF_PIN3_FPU_EVENTOUT = 15
}gpioe_af_pin3_t;

typedef enum
{
    GPIOE_AF_PIN4_TRACED1 = 0,
    GPIOE_AF_PIN4_SPI4_NSS = 5,
    GPIOE_AF_PIN4_FPU_EVENTOUT = 15
}gpioe_af_pin4_t;

typedef enum
{
    GPIOE_AF_PIN5_TRACED2 = 0,
    GPIOE_AF_PIN5_TIM9_CH1 = 3,
    GPIOE_AF_PIN5_SPI4_MISO = 5,
    GPIOE_AF_PIN5_FPU_EVENTOUT = 15
}gpioe_af_pin5_t;

typedef enum
{
    GPIOE_AF_PIN6_TRACED3 = 0,
    GPIOE_AF_PIN6_TIM9_CH2 = 3,
    GPIOE_AF_PIN6_SPI4_MOSI = 5,
    GPIOE_AF_PIN6_FPU_EVENTOUT = 15
}gpioe_af_pin6_t;

typedef enum
{
    GPIOE_AF_PIN7_TIM1_ETR = 1,
    GPIOE_AF_PIN7_FPU_EVENTOUT = 15
}gpioe_af_pin7_t;

typedef enum
{
    GPIOE_AF_PIN8_TIM1_CH1N = 1,
    GPIOE_AF_PIN8_FPU_EVENTOUT = 15
}gpioe_af_pin8_t;

typedef enum
{
    GPIOE_AF_PIN9_TIM1_CH1 = 1,
    GPIOE_AF_PIN9_FPU_EVENTOUT = 15
}gpioe_af_pin9_t;

typedef enum
{
    GPIOE_AF_PIN10_TIM1_CH2N = 1,
    GPIOE_AF_PIN10_FPU_EVENTOUT = 15
}gpioe_af_pin10_t;

typedef enum
{
    GPIOE_AF_PIN11_TIM1_CH2 = 1,
    GPIOE_AF_PIN11_SPI4_NSS = 5,
    GPIOE_AF_PIN11_FPU_EVENTOUT = 15
}gpioe_af_pin11_t;

typedef enum
{
    GPIOE_AF_PIN12_TIM1_CH3N = 1,
    GPIOE_AF_PIN12_SPI4_SCK = 5,
    GPIOE_AF_PIN12_FPU_EVENTOUT = 15
}gpioe_af_pin12_t;

typedef enum
{
    GPIOE_AF_PIN13_TIM1_CH3 = 1,
    GPIOE_AF_PIN13_SPI4_MISO = 5,
    GPIOE_AF_PIN13_FPU_EVENTOUT = 15
}gpioe_af_pin13_t;

typedef enum
{
    GPIOE_AF_PIN14_TIM1_CH4 = 1,
    GPIOE_AF_PIN14_SPI4_MOSI = 5,
    GPIOE_AF_PIN14_FPU_EVENTOUT = 15
}gpioe_af_pin14_t;

typedef enum
{
    GPIOE_AF_PIN15_TIM1_BKIN = 1,
    GPIOE_AF_PIN15_FPU_EVENTOUT = 15
}gpioe_af_pin15_t;

typedef enum
{
    GPIOH_AF_PIN0_FPU_EVENTOUT = 15
}gpioh_af_pin0_t;

typedef enum
{
    GPIOH_AF_PIN1_FPU_EVENTOUT = 15
}gpioh_af_pin1_t;
/*
 * END OF
 * Pins alternative functions for
 * STM32F401xB and STM32F401xC devices.
 */

/*
 * GPIO alternate function type.
 */
typedef union
{
    uint16_t value;
    union
    {
        gpioa_af_pin0_t pa0;
        gpioa_af_pin1_t pa1;
        gpioa_af_pin2_t pa2;
        gpioa_af_pin3_t pa3;
        gpioa_af_pin4_t pa4;
        gpioa_af_pin5_t pa5;
        gpioa_af_pin6_t pa6;
        gpioa_af_pin7_t pa7;
        gpioa_af_pin8_t pa8;
        gpioa_af_pin9_t pa9;
        gpioa_af_pin10_t pa10;
        gpioa_af_pin11_t pa11;
        gpioa_af_pin12_t pa12;
        gpioa_af_pin13_t pa13;
        gpioa_af_pin14_t pa14;
        gpioa_af_pin15_t pa15;
    }gpioa;
    union
    {
        gpiob_af_pin0_t pb0;
        gpiob_af_pin1_t pb1;
        gpiob_af_pin2_t pb2;
        gpiob_af_pin3_t pb3;
        gpiob_af_pin4_t pb4;
        gpiob_af_pin5_t pb5;
        gpiob_af_pin6_t pb6;
        gpiob_af_pin7_t pb7;
        gpiob_af_pin8_t pb8;
        gpiob_af_pin9_t pb9;
        gpiob_af_pin10_t pb10;
        gpiob_af_pin11_t pb11;
        gpiob_af_pin12_t pb12;
        gpiob_af_pin13_t pb13;
        gpiob_af_pin14_t pb14;
        gpiob_af_pin15_t pb15;
    }gpiob;
    union
    {
        gpioc_af_pin0_t pc0;
        gpioc_af_pin1_t pc1;
        gpioc_af_pin2_t pc2;
        gpioc_af_pin3_t pc3;
        gpioc_af_pin4_t pc4;
        gpioc_af_pin5_t pc5;
        gpioc_af_pin6_t pc6;
        gpioc_af_pin7_t pc7;
        gpioc_af_pin8_t pc8;
        gpioc_af_pin9_t pc9;
        gpioc_af_pin10_t pc10;
        gpioc_af_pin11_t pc11;
        gpioc_af_pin12_t pc12;
        gpioc_af_pin13_t pc13;
        gpioc_af_pin14_t pc14;
        gpioc_af_pin15_t pc15;
    }gpioc;
    union
    {
        gpiod_af_pin0_t pd0;
        gpiod_af_pin1_t pd1;
        gpiod_af_pin2_t pd2;
        gpiod_af_pin3_t pd3;
        gpiod_af_pin4_t pd4;
        gpiod_af_pin5_t pd5;
        gpiod_af_pin6_t pd6;
        gpiod_af_pin7_t pd7;
        gpiod_af_pin8_t pd8;
        gpiod_af_pin9_t pd9;
        gpiod_af_pin10_t pd10;
        gpiod_af_pin11_t pd11;
        gpiod_af_pin12_t pd12;
        gpiod_af_pin13_t pd13;
        gpiod_af_pin14_t pd14;
        gpiod_af_pin15_t pd15;
    }gpiod;
    union
    {
        gpioe_af_pin0_t pe0;
        gpioe_af_pin1_t pe1;
        gpioe_af_pin2_t pe2;
        gpioe_af_pin3_t pe3;
        gpioe_af_pin4_t pe4;
        gpioe_af_pin5_t pe5;
        gpioe_af_pin6_t pe6;
        gpioe_af_pin7_t pe7;
        gpioe_af_pin8_t pe8;
        gpioe_af_pin9_t pe9;
        gpioe_af_pin10_t pe10;
        gpioe_af_pin11_t pe11;
        gpioe_af_pin12_t pe12;
        gpioe_af_pin13_t pe13;
        gpioe_af_pin14_t pe14;
        gpioe_af_pin15_t pe15;
    }gpioe;
    union
    {
        gpioh_af_pin0_t ph0;
        gpioh_af_pin1_t ph1;
    }gpioh;
}gpio_af_t;


/*
 * @brief Sets GPIO mode.
 * @param port A port for which the mode shall be set.
 * @param pins A structure describing for which pins from the given port the mode shall be set.
 * @param mode A mode to be set.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If failed to set mode.
 * @retval CORE_EC_SUCCESS If succeeded to set mode.
 */
core_ec_t gpio_set_mode(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_mode_t mode);

/*
 * @brief Sets GPIO output type.
 * @param port A port for which the mode shall be set.
 * @param pins A structure describing for which pins from the given port the output type shall be set.
 * @param out_type An output type to be set.
 * @attention
 * Please note that the output type can only be changed when the pin mode is configured as either
 * output or alternate function. Any attempt to set output type for pins configured as input or analog
 * will produce an error.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If failed to set output type.
 * @retval CORE_EC_SUCCESS If succeeded to set output type.
 */
core_ec_t gpio_set_out_type(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_otype_t out_type);

/*
 * @brief Sets GPIO output speed.
 * @param port A port for which the mode shall be set.
 * @param pins A structure describing for which pins from the given port the output speed shall be set.
 * @param out_speed A speed to be set.
 * @attention
 * Please note that the speed can only be changed when the pin mode is configured as either
 * output or alternate function. Any attempt to set speed for pins configured as input or analog
 * will produce an error.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If failed to set output speed.
 * @retval CORE_EC_SUCCESS If succeeded to set output speed.
 */
core_ec_t gpio_set_out_speed(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_ospeed_t out_speed);

/*
 * @brief Sets GPIO pull type.
 * @param port A port for which the pull type shall be set.
 * @param pins A structure describing for which pins from the given port the pull type shall be set.
 * @param pull_type A pull type to be set.
 * @attention
 * It's not allowed to set pull type other than 0 (GPIO_PTYPE_NO_PULL) for the pins configured as analog.
 * Any attempt to do that will produce an error.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If failed to set pull type.
 * @retval CORE_EC_SUCCESS If succeeded to set pull type.
 */
core_ec_t gpio_set_pull_type(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_ptype_t pull_type);

/*
 * @brief Sets the data on the GPIO port with the value given.
 * @param port A port for which the data shall be set.
 * @param pins A structure describing for which pins the data shall be set.
 * @param data A data to set on the pins.
 * @attention
 * Please note that in order to set the data on the pin, the pin should be configured as output.
 * Any attempt to set data on the pin not configured as output will produce an error.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If failed to set data.
 * @retval CORE_EC_SUCCESS If succeeded to set data.
 */
core_ec_t gpio_set_data(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_pins_t data);

/*
 * @brief Gets the data from the port.
 * @param port A port for which the data shall be obtained.
 * @param data A pointer where the obtained data will be stored.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If failed to get data.
 * @retval CORE_EC_SUCCESS If succeeded to get data.
 */
core_ec_t gpio_get_data(
        gpio_port_t port,
        gpio_pins_t* data);

/*
 * @brief Locks gpio port configuration.
 * @param port A port for which the configuration shall be locked.
 * @param pins A structure describing which pins configuration shall be locked.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If failed to lock config.
 * @retval CORE_EC_SUCCESS If succeeded to lock config.
 */
core_ec_t gpio_lock_config(
        gpio_port_t port,
        gpio_pins_t pins);

/*
 * @brief Sets alternate function for the gpio pin.
 * @param port A port for which the alternate function shall be set.
 * @param pins A structure describing for which pin the alternate function shall be set.
 * @note
 * The function can only set alternate function for one pin at a time.
 * An attempt to set more than one will produce an error.
 * @param af An alternate function to set.
 * @note
 * User is responsible to select alternate function that makes sense for the selected pin.
 * Function will return error if the selected alternate function is not possible to set
 * for the given pin.
 * @returns core_ec_t
 * @retval CORE_EC_FAILED If failed to set alternate function.
 * @retval CORE_EC_SUCCESS If succeeded to set alternate function.
 */
core_ec_t gpio_set_alternate_function(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_af_t af);

#ifdef __cplusplus
}
#endif


#endif /* CORE_GPIO_H_ */
