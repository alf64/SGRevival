/*
 * gpio.c
 *
 *  Created on: 5 Jan 2022
 *      Author: alf64
 *
 *  Provides functionality for General Purpose I/O (GPIO)
 */


#include <core/gpio.h>
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
 * ---------- GPIO registers and bits definitions ----------
 */

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Each MODERx field configures direction mode of a gpio pin (x=0, ... , 15)
         * 00: Input (reset state)
         * 01: General purpose output mode.
         * 10: Alternate function mode.
         * 11: Analog mode.
         * Each field is RW.
         */
        uint32_t moder0 : 2;
        uint32_t moder1 : 2;
        uint32_t moder2 : 2;
        uint32_t moder3 : 2;
        uint32_t moder4 : 2;
        uint32_t moder5 : 2;
        uint32_t moder6 : 2;
        uint32_t moder7 : 2;
        uint32_t moder8 : 2;
        uint32_t moder9 : 2;
        uint32_t moder10 : 2;
        uint32_t moder11 : 2;
        uint32_t moder12 : 2;
        uint32_t moder13 : 2;
        uint32_t moder14 : 2;
        uint32_t moder15 : 2;
    }bits;
}gpio_reg_moder_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Each OTYPEx field configures output type of a gpio pin (x=0, ... , 15)
         * 0: Output push-pull.
         * 1: Output open-drain.
         * Each field is RW.
         */
        uint32_t ot0 : 1;
        uint32_t ot1 : 1;
        uint32_t ot2 : 1;
        uint32_t ot3 : 1;
        uint32_t ot4 : 1;
        uint32_t ot5 : 1;
        uint32_t ot6 : 1;
        uint32_t ot7 : 1;
        uint32_t ot8 : 1;
        uint32_t ot9 : 1;
        uint32_t ot10 : 1;
        uint32_t ot11 : 1;
        uint32_t ot12 : 1;
        uint32_t ot13 : 1;
        uint32_t ot14 : 1;
        uint32_t ot15 : 1;
        uint32_t rsvd16_31 : 16; // Reserved
    }bits;
}gpio_reg_otyper_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Each OSPEEDRx field configures output speed of a gpio pin (x=0, ... , 15)
         * 00: Low speed.
         * 01: Medium speed.
         * 10: High speed.
         * 11: Very High speed.
         * Each field is RW.
         */
        uint32_t ospeedr0 : 2;
        uint32_t ospeedr1 : 2;
        uint32_t ospeedr2 : 2;
        uint32_t ospeedr3 : 2;
        uint32_t ospeedr4 : 2;
        uint32_t ospeedr5 : 2;
        uint32_t ospeedr6 : 2;
        uint32_t ospeedr7 : 2;
        uint32_t ospeedr8 : 2;
        uint32_t ospeedr9 : 2;
        uint32_t ospeedr10 : 2;
        uint32_t ospeedr11 : 2;
        uint32_t ospeedr12 : 2;
        uint32_t ospeedr13 : 2;
        uint32_t ospeedr14 : 2;
        uint32_t ospeedr15 : 2;
    }bits;
}gpio_reg_ospeedr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Each PUPDRx field configures pull-up or pull-down of a gpio pin (x=0, ... , 15)
         * 00: No pull-up neither pull-down
         * 01: Pull-up.
         * 10: Pull-down.
         * 11: Reserved.
         * Each field is RW.
         */
        uint32_t pupdr0 : 2;
        uint32_t pupdr1 : 2;
        uint32_t pupdr2 : 2;
        uint32_t pupdr3 : 2;
        uint32_t pupdr4 : 2;
        uint32_t pupdr5 : 2;
        uint32_t pupdr6 : 2;
        uint32_t pupdr7 : 2;
        uint32_t pupdr8 : 2;
        uint32_t pupdr9 : 2;
        uint32_t pupdr10 : 2;
        uint32_t pupdr11 : 2;
        uint32_t pupdr12 : 2;
        uint32_t pupdr13 : 2;
        uint32_t pupdr14 : 2;
        uint32_t pupdr15 : 2;
    }bits;
}gpio_reg_pupdr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Each IDRx field shows the input state of the gpio pin (x=0, ... , 15)
         * Each field is Read-Only.
         */
        uint32_t idr0 : 1;
        uint32_t idr1 : 1;
        uint32_t idr2 : 1;
        uint32_t idr3 : 1;
        uint32_t idr4 : 1;
        uint32_t idr5 : 1;
        uint32_t idr6 : 1;
        uint32_t idr7 : 1;
        uint32_t idr8 : 1;
        uint32_t idr9 : 1;
        uint32_t idr10 : 1;
        uint32_t idr11 : 1;
        uint32_t idr12 : 1;
        uint32_t idr13 : 1;
        uint32_t idr14 : 1;
        uint32_t idr15 : 1;
        uint32_t rsvd16_31 : 16; // Reserved
    }bits;
}gpio_reg_idr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Each ODRx field allows for altering the output value of the gpio pin (x=0, ... , 15)
         * Each field is RW.
         */
        uint32_t odr0 : 1;
        uint32_t odr1 : 1;
        uint32_t odr2 : 1;
        uint32_t odr3 : 1;
        uint32_t odr4 : 1;
        uint32_t odr5 : 1;
        uint32_t odr6 : 1;
        uint32_t odr7 : 1;
        uint32_t odr8 : 1;
        uint32_t odr9 : 1;
        uint32_t odr10 : 1;
        uint32_t odr11 : 1;
        uint32_t odr12 : 1;
        uint32_t odr13 : 1;
        uint32_t odr14 : 1;
        uint32_t odr15 : 1;
        uint32_t rsvd16_31 : 16; // Reserved
    }bits;
}gpio_reg_odr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * These bits allows to set (set to 1) the gpio pin (x=0, ... , 15)
         * as if it was done by accessing the ODR register.
         * The difference is such operation is atomic and is never interrupted by
         * event/interrupt.
         * Each field is Write-Only. Read attempt will return 0.
         */
        uint32_t bs0 : 1;
        uint32_t bs1 : 1;
        uint32_t bs2 : 1;
        uint32_t bs3 : 1;
        uint32_t bs4 : 1;
        uint32_t bs5 : 1;
        uint32_t bs6 : 1;
        uint32_t bs7 : 1;
        uint32_t bs8 : 1;
        uint32_t bs9 : 1;
        uint32_t bs10 : 1;
        uint32_t bs11 : 1;
        uint32_t bs12 : 1;
        uint32_t bs13 : 1;
        uint32_t bs14 : 1;
        uint32_t bs15 : 1;
        /*
         * These bits allows to reset (set to 0) the gpio pin (x=0, ... , 15)
         * as if it was done by accessing the ODR register.
         * The difference is such operation is atomic and is never interrupted by
         * event/interrupt.
         * Each field is Write-Only. Read attempt will return 0.
         */
        uint32_t br0 : 1;
        uint32_t br1 : 1;
        uint32_t br2 : 1;
        uint32_t br3 : 1;
        uint32_t br4 : 1;
        uint32_t br5 : 1;
        uint32_t br6 : 1;
        uint32_t br7 : 1;
        uint32_t br8 : 1;
        uint32_t br9 : 1;
        uint32_t br10 : 1;
        uint32_t br11 : 1;
        uint32_t br12 : 1;
        uint32_t br13 : 1;
        uint32_t br14 : 1;
        uint32_t br15 : 1;
    }bits;
}gpio_reg_bsrr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Each LCKx field allows for gpio pin configuration lock.
         * This means the configuration of such pin cannot be altered
         * until the MCU reset or GPIO peripheral reset occurs.
         * Each field is RW and can only be written when lckk bit is 0.
         * 0 - configuration not locked.
         * 1 - configuration locked.
         * For the configuration lock to take effect, the lckk bit shall be set
         * to 1 using the lock key write sequence.
         */
        uint32_t lck0 : 1;
        uint32_t lck1 : 1;
        uint32_t lck2 : 1;
        uint32_t lck3 : 1;
        uint32_t lck4 : 1;
        uint32_t lck5 : 1;
        uint32_t lck6 : 1;
        uint32_t lck7 : 1;
        uint32_t lck8 : 1;
        uint32_t lck9 : 1;
        uint32_t lck10 : 1;
        uint32_t lck11 : 1;
        uint32_t lck12 : 1;
        uint32_t lck13 : 1;
        uint32_t lck14 : 1;
        uint32_t lck15 : 1;
        /*
         * Lock Key.
         * RW.
         * Read can be performed at any time, however write can only be performed using
         * the lock key write sequence.
         * The bit can have following values:
         * 0 - lock key not active.
         * 1 - lock key active.
         * The lock key write sequence is as follows:
         * LCKR_reg = (1<<lckk) | (lck0_15)
         * LCKR_reg = (0<<lckk) | (lck0_15)
         * LCKR_reg = (1<<lckk) | (lck0_15)
         * tmp = LCKR_reg
         * tmp = LCKR_reg and check if lckk is 1. This means lock is active.
         */
        uint32_t lckk : 1;
        uint32_t rsvd17_31 : 15; // Reserved
    }bits;
}gpio_reg_lckr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Each AFRLx fields allows for selecting gpio pin (x=0, ... , 7) alternate function.
         * Each field is RW.
         * Each field selects the Alternate Function (AF) for the pin:
         * 0000 - AF0
         * 0001 - AF1
         * ...
         * 1111 - AF15
         * Possible alternate function depends on the GPIO port and the GPIO pin.
         * The list of possible alternate function can be obtained from device datasheet,
         * in the pinouts/gpio description section (alternate function mapping).
         */
        uint32_t afrl0 : 4;
        uint32_t afrl1 : 4;
        uint32_t afrl2 : 4;
        uint32_t afrl3 : 4;
        uint32_t afrl4 : 4;
        uint32_t afrl5 : 4;
        uint32_t afrl6 : 4;
        uint32_t afrl7 : 4;
    }bits;
}gpio_reg_afrl_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Each AFRHx fields allows for selecting gpio pin (x=8, ... , 15) alternate function.
         * Each field is RW.
         * Each field selects the Alternate Function (AF) for the pin:
         * 0000 - AF0
         * 0001 - AF1
         * ...
         * 1111 - AF15
         * Possible alternate function depends on the GPIO port and the GPIO pin.
         * The list of possible alternate function can be obtained from device datasheet,
         * in the pinouts/gpio description section (alternate function mapping).
         */
        uint32_t afrh8 : 4;
        uint32_t afrh9 : 4;
        uint32_t afrh10 : 4;
        uint32_t afrh11 : 4;
        uint32_t afrh12 : 4;
        uint32_t afrh13 : 4;
        uint32_t afrh14 : 4;
        uint32_t afrh15 : 4;
    }bits;
}gpio_reg_afrh_t;

typedef union
{
    uint32_t regs_array[10];
    struct
    {
        /*
         * GPIO mode register.
         * Reset value (port A): 0x0c000000
         */
        gpio_reg_moder_t GPIOA_MODER;
        /*
         * GPIO output type register.
         * Reset value: 0x0.
         */
        gpio_reg_otyper_t GPIOA_OTYPER;
        /*
         * GPIO output speed register.
         * Reset value (port A): 0x0c000000
         */
        gpio_reg_ospeedr_t GPIOA_OSPEEDR;
        /*
         * GPIO pull-up/pull-down register.
         * Reset value (port A): 0x64000000
         */
        gpio_reg_pupdr_t GPIOA_PUPDR;
        /*
         * GPIO input data register.
         * Reset value: 0x0000XXXX
         * (X = undefined)
         */
        gpio_reg_idr_t GPIOA_IDR;
        /*
         * GPIO output data register.
         * Reset value: 0x0.
         */
        gpio_reg_odr_t GPIOA_ODR;
        /*
         * GPIO port bit set/reset register.
         * Reset value: 0x0.
         */
        gpio_reg_bsrr_t GPIOA_BSRR;
        /*
         * GPIO port configuration lock register.
         * A specific write (lock key) sequence is used to write this register.
         * During this sequence the value of LCKR[15:0] bits shall not change.
         * Only 32-bit long access is allowed during write sequence.
         * Reset value: 0x0.
         */
        gpio_reg_lckr_t GPIOA_LCKR;
        /*
         * GPIO alternate function low register.
         * Reset value: 0x0.
         */
        gpio_reg_afrl_t GPIOA_AFRL;
        /*
         * GPIO alternate function high register.
         * Reset value: 0x0.
         */
        gpio_reg_afrh_t GPIOA_AFRH;
    }regs;
}gpioa_regs_t;

typedef union
{
    uint32_t regs_array[10];
    struct
    {
        /*
         * GPIO mode register.
         * Reset value (port B): 0x00000280
         */
        gpio_reg_moder_t GPIOB_MODER;
        /*
         * GPIO output type register.
         * Reset value: 0x0.
         */
        gpio_reg_otyper_t GPIOB_OTYPER;
        /*
         * GPIO output speed register.
         * Reset value (port B): 0x000000c0
         */
        gpio_reg_ospeedr_t GPIOB_OSPEEDR;
        /*
         * GPIO pull-up/pull-down register.
         * Reset value (port B): 0x00000100
         */
        gpio_reg_pupdr_t GPIOB_PUPDR;
        /*
         * GPIO input data register.
         * Reset value: 0x0000XXXX
         * (X = undefined)
         */
        gpio_reg_idr_t GPIOB_IDR;
        /*
         * GPIO output data register.
         * Reset value: 0x0.
         */
        gpio_reg_odr_t GPIOB_ODR;
        /*
         * GPIO port bit set/reset register.
         * Reset value: 0x0.
         */
        gpio_reg_bsrr_t GPIOB_BSRR;
        /*
         * GPIO port configuration lock register.
         * A specific write (lock key) sequence is used to write this register.
         * During this sequence the value of LCKR[15:0] bits shall not change.
         * Only 32-bit long access is allowed during write sequence.
         * Reset value: 0x0.
         */
        gpio_reg_lckr_t GPIOB_LCKR;
        /*
         * GPIO alternate function low register.
         * Reset value: 0x0.
         */
        gpio_reg_afrl_t GPIOB_AFRL;
        /*
         * GPIO alternate function high register.
         * Reset value: 0x0.
         */
        gpio_reg_afrh_t GPIOB_AFRH;
    }regs;
}gpiob_regs_t;

typedef union
{
    uint32_t regs_array[10];
    struct
    {
        /*
         * GPIO mode register.
         * Reset value: 0x00000000
         */
        gpio_reg_moder_t GPIOC_MODER;
        /*
         * GPIO output type register.
         * Reset value: 0x0.
         */
        gpio_reg_otyper_t GPIOC_OTYPER;
        /*
         * GPIO output speed register.
         * Reset value: 0x00000000
         */
        gpio_reg_ospeedr_t GPIOC_OSPEEDR;
        /*
         * GPIO pull-up/pull-down register.
         * Reset value: 0x00000000
         */
        gpio_reg_pupdr_t GPIOC_PUPDR;
        /*
         * GPIO input data register.
         * Reset value: 0x0000XXXX
         * (X = undefined)
         */
        gpio_reg_idr_t GPIOC_IDR;
        /*
         * GPIO output data register.
         * Reset value: 0x0.
         */
        gpio_reg_odr_t GPIOC_ODR;
        /*
         * GPIO port bit set/reset register.
         * Reset value: 0x0.
         */
        gpio_reg_bsrr_t GPIOC_BSRR;
        /*
         * GPIO port configuration lock register.
         * A specific write (lock key) sequence is used to write this register.
         * During this sequence the value of LCKR[15:0] bits shall not change.
         * Only 32-bit long access is allowed during write sequence.
         * Reset value: 0x0.
         */
        gpio_reg_lckr_t GPIOC_LCKR;
        /*
         * GPIO alternate function low register.
         * Reset value: 0x0.
         */
        gpio_reg_afrl_t GPIOC_AFRL;
        /*
         * GPIO alternate function high register.
         * Reset value: 0x0.
         */
        gpio_reg_afrh_t GPIOC_AFRH;
    }regs;
}gpioc_regs_t;

typedef union
{
    uint32_t regs_array[10];
    struct
    {
        /*
         * GPIO mode register.
         * Reset value: 0x00000000
         */
        gpio_reg_moder_t GPIOD_MODER;
        /*
         * GPIO output type register.
         * Reset value: 0x0.
         */
        gpio_reg_otyper_t GPIOD_OTYPER;
        /*
         * GPIO output speed register.
         * Reset value: 0x00000000
         */
        gpio_reg_ospeedr_t GPIOD_OSPEEDR;
        /*
         * GPIO pull-up/pull-down register.
         * Reset value: 0x00000000
         */
        gpio_reg_pupdr_t GPIOD_PUPDR;
        /*
         * GPIO input data register.
         * Reset value: 0x0000XXXX
         * (X = undefined)
         */
        gpio_reg_idr_t GPIOD_IDR;
        /*
         * GPIO output data register.
         * Reset value: 0x0.
         */
        gpio_reg_odr_t GPIOD_ODR;
        /*
         * GPIO port bit set/reset register.
         * Reset value: 0x0.
         */
        gpio_reg_bsrr_t GPIOD_BSRR;
        /*
         * GPIO port configuration lock register.
         * A specific write (lock key) sequence is used to write this register.
         * During this sequence the value of LCKR[15:0] bits shall not change.
         * Only 32-bit long access is allowed during write sequence.
         * Reset value: 0x0.
         */
        gpio_reg_lckr_t GPIOD_LCKR;
        /*
         * GPIO alternate function low register.
         * Reset value: 0x0.
         */
        gpio_reg_afrl_t GPIOD_AFRL;
        /*
         * GPIO alternate function high register.
         * Reset value: 0x0.
         */
        gpio_reg_afrh_t GPIOD_AFRH;
    }regs;
}gpiod_regs_t;

typedef union
{
    uint32_t regs_array[10];
    struct
    {
        /*
         * GPIO mode register.
         * Reset value: 0x00000000
         */
        gpio_reg_moder_t GPIOE_MODER;
        /*
         * GPIO output type register.
         * Reset value: 0x0.
         */
        gpio_reg_otyper_t GPIOE_OTYPER;
        /*
         * GPIO output speed register.
         * Reset value: 0x00000000
         */
        gpio_reg_ospeedr_t GPIOE_OSPEEDR;
        /*
         * GPIO pull-up/pull-down register.
         * Reset value: 0x00000000
         */
        gpio_reg_pupdr_t GPIOE_PUPDR;
        /*
         * GPIO input data register.
         * Reset value: 0x0000XXXX
         * (X = undefined)
         */
        gpio_reg_idr_t GPIOE_IDR;
        /*
         * GPIO output data register.
         * Reset value: 0x0.
         */
        gpio_reg_odr_t GPIOE_ODR;
        /*
         * GPIO port bit set/reset register.
         * Reset value: 0x0.
         */
        gpio_reg_bsrr_t GPIOE_BSRR;
        /*
         * GPIO port configuration lock register.
         * A specific write (lock key) sequence is used to write this register.
         * During this sequence the value of LCKR[15:0] bits shall not change.
         * Only 32-bit long access is allowed during write sequence.
         * Reset value: 0x0.
         */
        gpio_reg_lckr_t GPIOE_LCKR;
        /*
         * GPIO alternate function low register.
         * Reset value: 0x0.
         */
        gpio_reg_afrl_t GPIOE_AFRL;
        /*
         * GPIO alternate function high register.
         * Reset value: 0x0.
         */
        gpio_reg_afrh_t GPIOE_AFRH;
    }regs;
}gpioe_regs_t;

typedef union
{
    uint32_t regs_array[10];
    struct
    {
        /*
         * GPIO mode register.
         * Reset value: 0x00000000
         */
        gpio_reg_moder_t GPIOH_MODER;
        /*
         * GPIO output type register.
         * Reset value: 0x0.
         */
        gpio_reg_otyper_t GPIOH_OTYPER;
        /*
         * GPIO output speed register.
         * Reset value: 0x00000000
         */
        gpio_reg_ospeedr_t GPIOH_OSPEEDR;
        /*
         * GPIO pull-up/pull-down register.
         * Reset value: 0x00000000
         */
        gpio_reg_pupdr_t GPIOH_PUPDR;
        /*
         * GPIO input data register.
         * Reset value: 0x0000XXXX
         * (X = undefined)
         */
        gpio_reg_idr_t GPIOH_IDR;
        /*
         * GPIO output data register.
         * Reset value: 0x0.
         */
        gpio_reg_odr_t GPIOH_ODR;
        /*
         * GPIO port bit set/reset register.
         * Reset value: 0x0.
         */
        gpio_reg_bsrr_t GPIOH_BSRR;
        /*
         * GPIO port configuration lock register.
         * A specific write (lock key) sequence is used to write this register.
         * During this sequence the value of LCKR[15:0] bits shall not change.
         * Only 32-bit long access is allowed during write sequence.
         * Reset value: 0x0.
         */
        gpio_reg_lckr_t GPIOH_LCKR;
        /*
         * GPIO alternate function low register.
         * Reset value: 0x0.
         */
        gpio_reg_afrl_t GPIOH_AFRL;
        /*
         * GPIO alternate function high register.
         * Reset value: 0x0.
         */
        gpio_reg_afrh_t GPIOH_AFRH;
    }regs;
}gpioh_regs_t;


static inline volatile gpioa_regs_t* gpioa_get_regs(void)
{
    return (volatile gpioa_regs_t*)(GPIOA_BASE);
}

static inline volatile gpiob_regs_t* gpiob_get_regs(void)
{
    return (volatile gpiob_regs_t*)(GPIOB_BASE);
}

static inline volatile gpioc_regs_t* gpioc_get_regs(void)
{
    return (volatile gpioc_regs_t*)(GPIOC_BASE);
}

static inline volatile gpiod_regs_t* gpiod_get_regs(void)
{
    return (volatile gpiod_regs_t*)(GPIOD_BASE);
}

static inline volatile gpioe_regs_t* gpioe_get_regs(void)
{
    return (volatile gpioe_regs_t*)(GPIOE_BASE);
}

static inline volatile gpioh_regs_t* gpioh_get_regs(void)
{
    return (volatile gpioh_regs_t*)(GPIOH_BASE);
}

/*
 * Types of regs for each GPIO port.
 */
typedef enum
{
    GPIO_REG_MODER = 0,
    GPIO_REG_OTYPER = 1,
    GPIO_REG_OSPEEDR = 2,
    GPIO_REG_PUPDR = 3,
    GPIO_REG_IDR = 4,
    GPIO_REG_ODR = 5,
    GPIO_REG_BSRR = 6,
    GPIO_REG_LCKR = 7,
    GPIO_REG_AFRL = 8,
    GPIO_REG_AFRH = 9
}gpio_regs_t;

/*
 * @brief Returns gpiox_regs_t* where x is your port letter, i.e. a.
 * @details
 * In fact it returns void*, you have to cast it to gpiox_regs_t* by yourself,
 * based on the port you passed to this function.
 * @returns pointer to gpio regs for the given port
 * @retval NULL means unable to obtain pointer to regs
 * @retval !NULL An actual pointer to regs for a given port.
 */
static volatile void* gpio_get_port_reg(
        gpio_port_t port,
        gpio_regs_t reg_type)
{
    volatile void* reg = (volatile void*)0xFFFFFFFF;

    // check reg_type correctness
    switch(reg_type)
    {
        case GPIO_REG_MODER:
        case GPIO_REG_OTYPER:
        case GPIO_REG_OSPEEDR:
        case GPIO_REG_PUPDR:
        case GPIO_REG_IDR:
        case GPIO_REG_ODR:
        case GPIO_REG_BSRR:
        case GPIO_REG_LCKR:
        case GPIO_REG_AFRL:
        case GPIO_REG_AFRH:
        {
            break;
        }
        default:
        {
            reg = NULL;
            break;
        }
    }
    if(reg == NULL)
    {
        return reg;
    }

    switch(port)
    {
        case GPIO_PORT_A:
        {
            volatile gpioa_regs_t* regs = (volatile void*)gpioa_get_regs();
            reg = (volatile void*)(&(regs->regs_array[reg_type]));

            break;
        }
        case GPIO_PORT_B:
        {
            volatile gpiob_regs_t* regs = (volatile void*)gpiob_get_regs();
            reg = (volatile void*)(&(regs->regs_array[reg_type]));

            break;
        }
        case GPIO_PORT_C:
        {
            volatile gpioc_regs_t* regs = (volatile void*)gpioc_get_regs();
            reg = (volatile void*)(&(regs->regs_array[reg_type]));

            break;
        }
        case GPIO_PORT_D:
        {
            volatile gpiod_regs_t* regs = (volatile void*)gpiod_get_regs();
            reg = (volatile void*)(&(regs->regs_array[reg_type]));

            break;
        }
        case GPIO_PORT_E:
        {
            volatile gpioe_regs_t* regs = (volatile void*)gpioe_get_regs();
            reg = (volatile void*)(&(regs->regs_array[reg_type]));

            break;
        }
        case GPIO_PORT_H:
        {
            volatile gpioh_regs_t* regs = (volatile void*)gpioh_get_regs();
            reg = (volatile void*)(&(regs->regs_array[reg_type]));

            break;
        }
        default:
        {
            reg = NULL;
            break;
        }
    }

    return reg;
}

core_ec_t gpio_set_mode(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_mode_t mode)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    // check pins correctness - at least one pin should be set
    if(pins.value == 0)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_moder_t* moder_ptr =
            (volatile gpio_reg_moder_t*)gpio_get_port_reg(port,  GPIO_REG_MODER);
    if(moder_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    // check mode correctness
    switch(mode)
    {
        case GPIO_MODE_INPUT:
        case GPIO_MODE_OUTPUT:
        case GPIO_MODE_ALTERNATE_FUNCTION:
        case GPIO_MODE_ANALOG_MODE:
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

    // set mode for selected pins
    gpio_reg_moder_t moder = *moder_ptr;
    uint8_t last_set_pin = 255;
    if(pins.pins.pin0)
    {
        last_set_pin = 0;
        moder.bits.moder0 = mode;
    }
    if(pins.pins.pin1)
    {
        last_set_pin = 1;
        moder.bits.moder1 = mode;
    }
    if(pins.pins.pin2)
    {
        last_set_pin = 2;
        moder.bits.moder2 = mode;
    }
    if(pins.pins.pin3)
    {
        last_set_pin = 3;
        moder.bits.moder3 = mode;
    }
    if(pins.pins.pin4)
    {
        last_set_pin = 4;
        moder.bits.moder4 = mode;
    }
    if(pins.pins.pin5)
    {
        last_set_pin = 5;
        moder.bits.moder5 = mode;
    }
    if(pins.pins.pin6)
    {
        last_set_pin = 6;
        moder.bits.moder6 = mode;
    }
    if(pins.pins.pin7)
    {
        last_set_pin = 7;
        moder.bits.moder7 = mode;
    }
    if(pins.pins.pin8)
    {
        last_set_pin = 8;
        moder.bits.moder8 = mode;
    }
    if(pins.pins.pin9)
    {
        last_set_pin = 9;
        moder.bits.moder9 = mode;
    }
    if(pins.pins.pin10)
    {
        last_set_pin = 10;
        moder.bits.moder10 = mode;
    }
    if(pins.pins.pin11)
    {
        last_set_pin = 11;
        moder.bits.moder11 = mode;
    }
    if(pins.pins.pin12)
    {
        last_set_pin = 12;
        moder.bits.moder12 = mode;
    }
    if(pins.pins.pin13)
    {
        last_set_pin = 13;
        moder.bits.moder13 = mode;
    }
    if(pins.pins.pin14)
    {
        last_set_pin = 14;
        moder.bits.moder14 = mode;
    }
    if(pins.pins.pin15)
    {
        last_set_pin = 15;
        moder.bits.moder15 = mode;
    }

    if(last_set_pin == 255)
    {
        return CORE_EC_FAILED;
    }

    // save the results into the register
    *moder_ptr = moder;

    // check if at least mode for first selected pin is set
    uint8_t moder_field_shift = (uint8_t)(last_set_pin * 2); // (moder has 32 bits usable, while pins only 16)
    do
    {
        moder = *moder_ptr;
    }while(((moder.value >> moder_field_shift) & 0x3) != mode);

    return ec;
}

core_ec_t gpio_set_out_type(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_otype_t out_type)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    // check pins correctness - at least one pin should be set
    if(pins.value == 0)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_otyper_t* otyper_ptr =
            (volatile gpio_reg_otyper_t*)gpio_get_port_reg(port,  GPIO_REG_OTYPER);
    if(otyper_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_moder_t* moder_ptr =
            (volatile gpio_reg_moder_t*)gpio_get_port_reg(port,  GPIO_REG_MODER);
    if(moder_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    // check out_type correctness
    switch(out_type)
    {
        case GPIO_OTYPE_PUSH_PULL:
        case GPIO_OTYPE_OPEN_DRAIN:
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

    // set out_type for selected pins
    gpio_reg_otyper_t otyper = *otyper_ptr;
    gpio_reg_moder_t moder = *moder_ptr;
    uint8_t last_set_pin = 255;
    if(pins.pins.pin0)
    {
        last_set_pin = 0;
        if(moder.bits.moder0 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder0 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot0 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin1)
    {
        last_set_pin = 1;
        if(moder.bits.moder1 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder1 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot1 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin2)
    {
        last_set_pin = 2;
        if(moder.bits.moder2 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder2 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot2 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }

    }
    if(pins.pins.pin3)
    {
        last_set_pin = 3;
        if(moder.bits.moder3 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder3 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot3 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin4)
    {
        last_set_pin = 4;
        if(moder.bits.moder4 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder4 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot4 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin5)
    {
        last_set_pin = 5;
        if(moder.bits.moder5 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder5 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot5 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin6)
    {
        last_set_pin = 6;
        if(moder.bits.moder6 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder6 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot6 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin7)
    {
        last_set_pin = 7;
        if(moder.bits.moder7 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder7 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot7 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin8)
    {
        last_set_pin = 8;
        if(moder.bits.moder8 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder8 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot8 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin9)
    {
        last_set_pin = 9;
        if(moder.bits.moder9 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder9 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot9 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin10)
    {
        last_set_pin = 10;
        if(moder.bits.moder10 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder10 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot10 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin11)
    {
        last_set_pin = 11;
        if(moder.bits.moder11 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder11 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot11 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin12)
    {
        last_set_pin = 12;
        if(moder.bits.moder12 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder12 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot12 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin13)
    {
        last_set_pin = 13;
        if(moder.bits.moder13 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder13 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot13 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin14)
    {
        last_set_pin = 14;
        if(moder.bits.moder14 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder14 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot14 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin15)
    {
        last_set_pin = 15;
        if(moder.bits.moder15 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder15 == GPIO_MODE_OUTPUT)
        {
            otyper.bits.ot15 = out_type;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }

    if(last_set_pin == 255)
    {
        return CORE_EC_FAILED;
    }

    // save the results into the register
    *otyper_ptr = otyper;

    // check if at least out_type for first selected pin is set
    do
    {
        otyper = *otyper_ptr;
    }while(((otyper.value >> last_set_pin) & 0x1) != out_type);

    return ec;
}

core_ec_t gpio_set_out_speed(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_ospeed_t out_speed)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    // check pins correctness - at least one pin should be set
    if(pins.value == 0)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_ospeedr_t* ospeedr_ptr =
            (volatile gpio_reg_ospeedr_t*)gpio_get_port_reg(port,  GPIO_REG_OSPEEDR);
    if(ospeedr_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_moder_t* moder_ptr =
            (volatile gpio_reg_moder_t*)gpio_get_port_reg(port,  GPIO_REG_MODER);
    if(moder_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    // check out_speed correctness
    switch(out_speed)
    {
        case GPIO_OSPEED_LOW:
        case GPIO_OSPEED_MEDIUM:
        case GPIO_OSPEED_HIGH:
        case GPIO_OSPEED_VERY_HIGH:
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

    // set out_speed for selected pins
    gpio_reg_ospeedr_t ospeedr = *ospeedr_ptr;
    gpio_reg_moder_t moder = *moder_ptr;
    uint8_t last_set_pin = 255;
    if(pins.pins.pin0)
    {
        last_set_pin = 0;
        if(moder.bits.moder0 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder0 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr0 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin1)
    {
        last_set_pin = 1;
        if(moder.bits.moder1 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder1 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr1 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin2)
    {
        last_set_pin = 2;
        if(moder.bits.moder2 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder2 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr2 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }

    }
    if(pins.pins.pin3)
    {
        last_set_pin = 3;
        if(moder.bits.moder3 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder3 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr3 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin4)
    {
        last_set_pin = 4;
        if(moder.bits.moder4 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder4 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr4 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin5)
    {
        last_set_pin = 5;
        if(moder.bits.moder5 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder5 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr5 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin6)
    {
        last_set_pin = 6;
        if(moder.bits.moder6 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder6 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr6 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin7)
    {
        last_set_pin = 7;
        if(moder.bits.moder7 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder7 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr7 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin8)
    {
        last_set_pin = 8;
        if(moder.bits.moder8 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder8 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr8 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin9)
    {
        last_set_pin = 9;
        if(moder.bits.moder9 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder9 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr9 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin10)
    {
        last_set_pin = 10;
        if(moder.bits.moder10 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder10 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr10 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin11)
    {
        last_set_pin = 11;
        if(moder.bits.moder11 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder11 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr11 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin12)
    {
        last_set_pin = 12;
        if(moder.bits.moder12 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder12 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr12 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin13)
    {
        last_set_pin = 13;
        if(moder.bits.moder13 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder13 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr13 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin14)
    {
        last_set_pin = 14;
        if(moder.bits.moder14 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder14 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr14 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin15)
    {
        last_set_pin = 15;
        if(moder.bits.moder15 == GPIO_MODE_ALTERNATE_FUNCTION ||
                moder.bits.moder15 == GPIO_MODE_OUTPUT)
        {
            ospeedr.bits.ospeedr15 = out_speed;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }

    if(last_set_pin == 255)
    {
        return CORE_EC_FAILED;
    }

    // save the results into the register
    *ospeedr_ptr = ospeedr;

    // check if at least out_speed for first selected pin is set
    uint8_t ospeedr_field_shift = (uint8_t)(last_set_pin * 2); // (ospeedr has 32 bits usable, while pins only 16)
    do
    {
        ospeedr = *ospeedr_ptr;
    }while(((ospeedr.value >> ospeedr_field_shift) & 0x3) != out_speed);

    return ec;
}

core_ec_t gpio_set_pull_type(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_ptype_t pull_type)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    // check pins correctness - at least one pin should be set
    if(pins.value == 0)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_pupdr_t* pupdr_ptr =
            (volatile gpio_reg_pupdr_t*)gpio_get_port_reg(port,  GPIO_REG_PUPDR);
    if(pupdr_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_moder_t* moder_ptr =
            (volatile gpio_reg_moder_t*)gpio_get_port_reg(port,  GPIO_REG_MODER);
    if(moder_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    // check pull_type correctness
    switch(pull_type)
    {
        case GPIO_PTYPE_NO_PULL:
        case GPIO_PTYPE_PULL_UP:
        case GPIO_PTYPE_PULL_DOWN:
        {
            break;
        }
        case GPIO_PTYPE_RESERVED:
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

    // set pull_type for selected pins
    gpio_reg_pupdr_t pupdr = *pupdr_ptr;
    gpio_reg_moder_t moder = *moder_ptr;
    uint8_t last_set_pin = 255;
    if(pins.pins.pin0)
    {
        last_set_pin = 0;
        if((moder.bits.moder0 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr0 = pull_type;
        }
    }
    if(pins.pins.pin1)
    {
        last_set_pin = 1;
        if((moder.bits.moder1 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr1 = pull_type;
        }
    }
    if(pins.pins.pin2)
    {
        last_set_pin = 2;
        if((moder.bits.moder2 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr2 = pull_type;
        }
    }
    if(pins.pins.pin3)
    {
        last_set_pin = 3;
        if((moder.bits.moder3 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr3 = pull_type;
        }
    }
    if(pins.pins.pin4)
    {
        last_set_pin = 4;
        if((moder.bits.moder4 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr4 = pull_type;
        }
    }
    if(pins.pins.pin5)
    {
        last_set_pin = 5;
        if((moder.bits.moder5 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr5 = pull_type;
        }
    }
    if(pins.pins.pin6)
    {
        last_set_pin = 6;
        if((moder.bits.moder6 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr6 = pull_type;
        }
    }
    if(pins.pins.pin7)
    {
        last_set_pin = 7;
        if((moder.bits.moder7 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr7 = pull_type;
        }
    }
    if(pins.pins.pin8)
    {
        last_set_pin = 8;
        if((moder.bits.moder8 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr8 = pull_type;
        }
    }
    if(pins.pins.pin9)
    {
        last_set_pin = 9;
        if((moder.bits.moder9 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr9 = pull_type;
        }
    }
    if(pins.pins.pin10)
    {
        last_set_pin = 10;
        if((moder.bits.moder10 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr10 = pull_type;
        }
    }
    if(pins.pins.pin11)
    {
        last_set_pin = 11;
        if((moder.bits.moder11 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr11 = pull_type;
        }
    }
    if(pins.pins.pin12)
    {
        last_set_pin = 12;
        if((moder.bits.moder12 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr12 = pull_type;
        }
    }
    if(pins.pins.pin13)
    {
        last_set_pin = 13;
        if((moder.bits.moder13 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr13 = pull_type;
        }
    }
    if(pins.pins.pin14)
    {
        last_set_pin = 14;
        if((moder.bits.moder14 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr14 = pull_type;
        }
    }
    if(pins.pins.pin15)
    {
        last_set_pin = 15;
        if((moder.bits.moder15 == GPIO_MODE_ANALOG_MODE) &&
                (pull_type != GPIO_PTYPE_NO_PULL))
        {
            return CORE_EC_FAILED;
        }
        else
        {
            pupdr.bits.pupdr15 = pull_type;
        }
    }

    if(last_set_pin == 255)
    {
        return CORE_EC_FAILED;
    }

    // save the results into the register
    *pupdr_ptr = pupdr;

    // check if at least pull_type for first selected pin is set
    uint8_t pupdr_field_shift = (uint8_t)(last_set_pin * 2); // (pupdr has 32 bits usable, while pins only 16)
    do
    {
        pupdr = *pupdr_ptr;
    }while(((pupdr.value >> pupdr_field_shift) & 0x3) != pull_type);

    return ec;
}

core_ec_t gpio_set_data(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_pins_t data)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    // check pins correctness - at least one pin should be set
    if(pins.value == 0)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_odr_t* odr_ptr =
            (volatile gpio_reg_odr_t*)gpio_get_port_reg(port,  GPIO_REG_ODR);
    if(odr_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_moder_t* moder_ptr =
            (volatile gpio_reg_moder_t*)gpio_get_port_reg(port,  GPIO_REG_MODER);
    if(moder_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    // set data for selected pins
    gpio_reg_odr_t odr = *odr_ptr;
    gpio_reg_moder_t moder = *moder_ptr;
    uint8_t last_set_pin = 255;
    if(pins.pins.pin0)
    {
        last_set_pin = 0;
        if(moder.bits.moder0 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr0 = data.pins.pin0;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin1)
    {
        last_set_pin = 1;
        if(moder.bits.moder1 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr1 = data.pins.pin1;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin2)
    {
        last_set_pin = 2;
        if(moder.bits.moder2 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr2 = data.pins.pin2;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin3)
    {
        last_set_pin = 3;
        if(moder.bits.moder3 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr3 = data.pins.pin3;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin4)
    {
        last_set_pin = 4;
        if(moder.bits.moder4 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr4 = data.pins.pin4;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin5)
    {
        last_set_pin = 5;
        if(moder.bits.moder5 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr5 = data.pins.pin5;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin6)
    {
        last_set_pin = 6;
        if(moder.bits.moder6 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr6 = data.pins.pin6;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin7)
    {
        last_set_pin = 7;
        if(moder.bits.moder7 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr7 = data.pins.pin7;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin8)
    {
        last_set_pin = 8;
        if(moder.bits.moder8 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr8 = data.pins.pin8;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin9)
    {
        last_set_pin = 9;
        if(moder.bits.moder9 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr9 = data.pins.pin9;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin10)
    {
        last_set_pin = 10;
        if(moder.bits.moder10 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr10 = data.pins.pin10;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin11)
    {
        last_set_pin = 11;
        if(moder.bits.moder11 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr11 = data.pins.pin11;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin12)
    {
        last_set_pin = 12;
        if(moder.bits.moder12 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr12 = data.pins.pin12;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin13)
    {
        last_set_pin = 13;
        if(moder.bits.moder13 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr13 = data.pins.pin13;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin14)
    {
        last_set_pin = 14;
        if(moder.bits.moder14 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr14 = data.pins.pin14;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }
    if(pins.pins.pin15)
    {
        last_set_pin = 15;
        if(moder.bits.moder15 == GPIO_MODE_OUTPUT)
        {
            odr.bits.odr15 = data.pins.pin15;
        }
        else
        {
            return CORE_EC_FAILED;
        }
    }

    if(last_set_pin == 255)
    {
        return CORE_EC_FAILED;
    }

    // save the results into the register
    *odr_ptr = odr;

    // check if at least data for first selected pin is set
    do
    {
        odr = *odr_ptr;
    }while(((odr.value >> last_set_pin) & 0x1) != ((data.value >> last_set_pin) & 0x1));

    return ec;
}

core_ec_t gpio_get_data(
        gpio_port_t port,
        gpio_pins_t* data)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    if(data == NULL)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_idr_t* idr_ptr =
            (volatile gpio_reg_idr_t*)gpio_get_port_reg(port,  GPIO_REG_IDR);
    if(idr_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    data->value = (uint16_t)(idr_ptr->value & 0xFFFF);

    return ec;
}

core_ec_t gpio_lock_config(
        gpio_port_t port,
        gpio_pins_t pins)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    // check pins correctness - at least one pin should be set
    if(pins.value == 0)
    {
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_lckr_t* lckr_ptr =
            (volatile gpio_reg_lckr_t*)gpio_get_port_reg(port,  GPIO_REG_LCKR);
    if(lckr_ptr == NULL)
    {
        return CORE_EC_FAILED;
    }

    gpio_reg_lckr_t lckr = *lckr_ptr;
    lckr.bits.lck0 = pins.pins.pin0;
    lckr.bits.lck1 = pins.pins.pin1;
    lckr.bits.lck2 = pins.pins.pin2;
    lckr.bits.lck3 = pins.pins.pin3;
    lckr.bits.lck4 = pins.pins.pin4;
    lckr.bits.lck5 = pins.pins.pin5;
    lckr.bits.lck6 = pins.pins.pin6;
    lckr.bits.lck7 = pins.pins.pin7;
    lckr.bits.lck8 = pins.pins.pin8;
    lckr.bits.lck9 = pins.pins.pin9;
    lckr.bits.lck10 = pins.pins.pin10;
    lckr.bits.lck11 = pins.pins.pin11;
    lckr.bits.lck12 = pins.pins.pin12;
    lckr.bits.lck13 = pins.pins.pin13;
    lckr.bits.lck14 = pins.pins.pin14;
    lckr.bits.lck15 = pins.pins.pin15;

    // perform lock sequence
    lckr.bits.lckk = 1;
    *lckr_ptr = lckr;
    lckr.bits.lckk = 0;
    *lckr_ptr = lckr;
    lckr.bits.lckk = 1;
    *lckr_ptr = lckr;

    // check if lock is active (upon 2nd read)
    lckr = *lckr_ptr;
    lckr = *lckr_ptr;
    if(lckr.bits.lckk != 1)
    {
        ec = CORE_EC_FAILED;
    }

    return ec;
}

core_ec_t gpio_set_alternate_function(
        gpio_port_t port,
        gpio_pins_t pins,
        gpio_af_t af)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    // check pins correctness - only one pin should be set
    bool pin_no = 255;
    for(uint8_t i = 0; i < 16; i++)
    {
        if(((pins.value >> i) & 0x1) == 1)
        {
            if(pin_no == 255)
            {
                // found selected pin
                pin_no = i;
            }
            else
            {
                // found another selected pin, thats an error!
                pin_no = 255;
                break;
            }
        }
    }
    if(pin_no == 255)
    {
        // either no selected pins or more than 1 selected
        return CORE_EC_FAILED;
    }

    volatile gpio_reg_afrl_t* afrl_ptr =
            (volatile gpio_reg_afrl_t*)gpio_get_port_reg(port,  GPIO_REG_AFRL);
    volatile gpio_reg_afrh_t* afrh_ptr =
            (volatile gpio_reg_afrh_t*)gpio_get_port_reg(port,  GPIO_REG_AFRH);
    if((afrl_ptr == NULL) || (afrh_ptr == NULL))
    {
        return CORE_EC_FAILED;
    }

    gpio_reg_afrl_t afrl = *afrl_ptr;
    gpio_reg_afrh_t afrh = *afrh_ptr;

    // check for AF function selection correctness
    switch(port)
    {
        case GPIO_PORT_A:
        {
            switch(pin_no)
            {
                case 0:
                {
                    switch(af.gpioa.pa0)
                    {
                        case GPIOA_AF_PIN0_TIM2_CH1_TIM2_ETR:
                        case GPIOA_AF_PIN0_TIM5_CH1:
                        case GPIOA_AF_PIN0_USART2_CTS:
                        case GPIOA_AF_PIN0_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl0 = af.gpioa.pa0;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl0 != af.gpioa.pa0);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa0

                    break;
                }
                case 1:
                {
                    switch(af.gpioa.pa1)
                    {
                        case GPIOA_AF_PIN1_TIM2_CH2:
                        case GPIOA_AF_PIN1_TIM5_CH2:
                        case GPIOA_AF_PIN1_USART2_RTS:
                        case GPIOA_AF_PIN1_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl1 = af.gpioa.pa1;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl1 != af.gpioa.pa1);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa1

                    break;
                }
                case 2:
                {
                    switch(af.gpioa.pa2)
                    {
                        case GPIOA_AF_PIN2_TIM2_CH3:
                        case GPIOA_AF_PIN2_TIM5_CH3:
                        case GPIOA_AF_PIN2_TIM9_CH1:
                        case GPIOA_AF_PIN2_USART2_TX:
                        case GPIOA_AF_PIN2_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl2 = af.gpioa.pa2;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl2 != af.gpioa.pa2);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa2

                    break;
                }
                case 3:
                {
                    switch(af.gpioa.pa3)
                    {
                        case GPIOA_AF_PIN3_TIM2_CH4:
                        case GPIOA_AF_PIN3_TIM5_CH4:
                        case GPIOA_AF_PIN3_TIM9_CH2:
                        case GPIOA_AF_PIN3_USART2_RX:
                        case GPIOA_AF_PIN3_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl3 = af.gpioa.pa3;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl3 != af.gpioa.pa3);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa3

                    break;
                }
                case 4:
                {
                    switch(af.gpioa.pa4)
                    {
                        case GPIOA_AF_PIN4_SPI1_NSS:
                        case GPIOA_AF_PIN4_SPI3_NSS_I2S3_WS:
                        case GPIOA_AF_PIN4_USART2_CK:
                        case GPIOA_AF_PIN4_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl4 = af.gpioa.pa4;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl4 != af.gpioa.pa4);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa4

                    break;
                }
                case 5:
                {
                    switch(af.gpioa.pa5)
                    {
                        case GPIOA_AF_PIN5_TIM2_CH1_TIM2_ETR:
                        case GPIOA_AF_PIN5_SPI1_SCK:
                        case GPIOA_AF_PIN5_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl5 = af.gpioa.pa5;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl5 != af.gpioa.pa5);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa5

                    break;
                }
                case 6:
                {
                    switch(af.gpioa.pa6)
                    {
                        case GPIOA_AF_PIN6_TIM1_BKIN:
                        case GPIOA_AF_PIN6_TIM3_CH1:
                        case GPIOA_AF_PIN6_SPI1_MISO:
                        case GPIOA_AF_PIN6_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl6 = af.gpioa.pa6;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl6 != af.gpioa.pa6);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa6

                    break;
                }
                case 7:
                {
                    switch(af.gpioa.pa7)
                    {
                        case GPIOA_AF_PIN7_TIM1_CH1N:
                        case GPIOA_AF_PIN7_TIM3_CH2:
                        case GPIOA_AF_PIN7_SPI1_MOSI:
                        case GPIOA_AF_PIN7_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl7 = af.gpioa.pa7;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl7 != af.gpioa.pa7);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa7

                    break;
                }
                case 8:
                {
                    switch(af.gpioa.pa8)
                    {
                        case GPIOA_AF_PIN8_MCO_1:
                        case GPIOA_AF_PIN8_TIM1_CH1:
                        case GPIOA_AF_PIN8_I2C3_SCL:
                        case GPIOA_AF_PIN8_USART1_CK:
                        case GPIOA_AF_PIN8_OTG_FS_SOF:
                        case GPIOA_AF_PIN8_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh8 = af.gpioa.pa8;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh8 != af.gpioa.pa8);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa8

                    break;
                }
                case 9:
                {
                    switch(af.gpioa.pa9)
                    {
                        case GPIOA_AF_PIN9_TIM1_CH2:
                        case GPIOA_AF_PIN9_I2C3_SMBA:
                        case GPIOA_AF_PIN9_USART1_TX:
                        case GPIOA_AF_PIN9_OTG_FS_VBUS:
                        case GPIOA_AF_PIN9_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh9 = af.gpioa.pa9;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh9 != af.gpioa.pa9);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa9

                    break;
                }
                case 10:
                {
                    switch(af.gpioa.pa10)
                    {
                        case GPIOA_AF_PIN10_TIM1_CH3:
                        case GPIOA_AF_PIN10_USART1_RX:
                        case GPIOA_AF_PIN10_OTG_FS_ID:
                        case GPIOA_AF_PIN10_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh10 = af.gpioa.pa10;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh10 != af.gpioa.pa10);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa10

                    break;
                }
                case 11:
                {
                    switch(af.gpioa.pa11)
                    {
                        case GPIOA_AF_PIN11_TIM1_CH4:
                        case GPIOA_AF_PIN11_USART1_CTS:
                        case GPIOA_AF_PIN11_USART6_TX:
                        case GPIOA_AF_PIN11_OTG_FS_DM:
                        case GPIOA_AF_PIN11_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh11 = af.gpioa.pa11;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh11 != af.gpioa.pa11);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa11

                    break;
                }
                case 12:
                {
                    switch(af.gpioa.pa12)
                    {
                        case GPIOA_AF_PIN12_TIM1_ETR:
                        case GPIOA_AF_PIN12_USART1_RTS:
                        case GPIOA_AF_PIN12_USART6_RX:
                        case GPIOA_AF_PIN12_OTG_FS_DP:
                        case GPIOA_AF_PIN12_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh12 = af.gpioa.pa12;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh12 != af.gpioa.pa12);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa12

                    break;
                }
                case 13:
                {
                    switch(af.gpioa.pa13)
                    {
                        case GPIOA_AF_PIN13_JTMS_SWDIO:
                        case GPIOA_AF_PIN13_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh13 = af.gpioa.pa13;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh13 != af.gpioa.pa13);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa13

                    break;
                }
                case 14:
                {
                    switch(af.gpioa.pa14)
                    {
                        case GPIOA_AF_PIN14_JTCK_SWCLK:
                        case GPIOA_AF_PIN14_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh14 = af.gpioa.pa14;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh14 != af.gpioa.pa14);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pa14

                    break;
                }
                case 15:
                {
                    switch(af.gpioa.pa15)
                    {
                        case GPIOA_AF_PIN15_JTDI:
                        case GPIOA_AF_PIN15_TIM2_CH1_TIM2_ETR:
                        case GPIOA_AF_PIN15_SPI1_NSS:
                        case GPIOA_AF_PIN15_SPI3_NSS_I2S3_WS:
                        case GPIOA_AF_PIN15_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh15 = af.gpioa.pa15;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh15 != af.gpioa.pa15);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    }

                    break;
                }
                default:
                {
                    ec = CORE_EC_FAILED;
                    break;
                }
            } // switch pin_no

            break;
        }
        case GPIO_PORT_B:
        {
            switch(pin_no)
            {
                case 0:
                {
                    switch(af.gpiob.pb0)
                    {
                        case GPIOB_AF_PIN0_TIM1_CH2N:
                        case GPIOB_AF_PIN0_TIM3_CH3:
                        case GPIOB_AF_PIN0_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl0 = af.gpiob.pb0;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl0 != af.gpiob.pb0);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb0

                    break;
                }
                case 1:
                {
                    switch(af.gpiob.pb1)
                    {
                        case GPIOB_AF_PIN1_TIM1_CH3N:
                        case GPIOB_AF_PIN1_TIM3_CH4:
                        case GPIOB_AF_PIN1_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl1 = af.gpiob.pb1;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl1 != af.gpiob.pb1);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb1

                    break;
                }
                case 2:
                {
                    switch(af.gpiob.pb2)
                    {
                        case GPIOB_AF_PIN2_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl2 = af.gpiob.pb2;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl2 != af.gpiob.pb2);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb2

                    break;
                }
                case 3:
                {
                    switch(af.gpiob.pb3)
                    {
                        case GPIOB_AF_PIN3_JTDO_SWO:
                        case GPIOB_AF_PIN3_TIM2_CH2:
                        case GPIOB_AF_PIN3_SPI1_SCK:
                        case GPIOB_AF_PIN3_SPI3_SCK_I2S3_CK:
                        case GPIOB_AF_PIN3_I2C2_SDA:
                        case GPIOB_AF_PIN3_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl3 = af.gpiob.pb3;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl3 != af.gpiob.pb3);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb3

                    break;
                }
                case 4:
                {
                    switch(af.gpiob.pb4)
                    {
                        case GPIOB_AF_PIN4_JTRST:
                        case GPIOB_AF_PIN4_TIM3_CH1:
                        case GPIOB_AF_PIN4_SPI1_MISO:
                        case GPIOB_AF_PIN4_SPI3_MISO:
                        case GPIOB_AF_PIN4_I2S3ext_SD:
                        case GPIOB_AF_PIN4_I2C3_SDA:
                        case GPIOB_AF_PIN4_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl4 = af.gpiob.pb4;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl4 != af.gpiob.pb4);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb4

                    break;
                }
                case 5:
                {
                    switch(af.gpiob.pb5)
                    {
                        case GPIOB_AF_PIN5_TIM3_CH2:
                        case GPIOB_AF_PIN5_I2C1_SMBA:
                        case GPIOB_AF_PIN5_SPI1_MOSI:
                        case GPIOB_AF_PIN5_SPI3_MOSI_I2S3_SD:
                        case GPIOB_AF_PIN5_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl5 = af.gpiob.pb5;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl5 != af.gpiob.pb5);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb5

                    break;
                }
                case 6:
                {
                    switch(af.gpiob.pb6)
                    {
                        case GPIOB_AF_PIN6_TIM4_CH1:
                        case GPIOB_AF_PIN6_I2C1_SCL:
                        case GPIOB_AF_PIN6_USART1_TX:
                        case GPIOB_AF_PIN6_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl6 = af.gpiob.pb6;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl6 != af.gpiob.pb6);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb6

                    break;
                }
                case 7:
                {
                    switch(af.gpiob.pb7)
                    {
                        case GPIOB_AF_PIN7_TIM4_CH2:
                        case GPIOB_AF_PIN7_I2C1_SDA:
                        case GPIOB_AF_PIN7_USART1_RX:
                        case GPIOB_AF_PIN7_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl7 = af.gpiob.pb7;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl7 != af.gpiob.pb7);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb7

                    break;
                }
                case 8:
                {
                    switch(af.gpiob.pb8)
                    {
                        case GPIOB_AF_PIN8_TIM4_CH3:
                        case GPIOB_AF_PIN8_TIM10_CH1:
                        case GPIOB_AF_PIN8_I2C1_SCL:
                        case GPIOB_AF_PIN8_SDIO_D4:
                        case GPIOB_AF_PIN8_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh8 = af.gpiob.pb8;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh8 != af.gpiob.pb8);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb8

                    break;
                }
                case 9:
                {
                    switch(af.gpiob.pb9)
                    {
                        case GPIOB_AF_PIN9_TIM4_CH4:
                        case GPIOB_AF_PIN9_TIM11_CH1:
                        case GPIOB_AF_PIN9_I2C1_SDA:
                        case GPIOB_AF_PIN9_SPI2_NSS_I2S2_WS:
                        case GPIOB_AF_PIN9_SDIO_D5:
                        case GPIOB_AF_PIN9_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh9 = af.gpiob.pb9;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh9 != af.gpiob.pb9);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb9

                    break;
                }
                case 10:
                {
                    switch(af.gpiob.pb10)
                    {
                        case GPIOB_AF_PIN10_TIM2_CH3:
                        case GPIOB_AF_PIN10_I2C2_SCL:
                        case GPIOB_AF_PIN10_SPI2_SCK_I2S2_CK:
                        case GPIOB_AF_PIN10_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh10 = af.gpiob.pb10;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh10 != af.gpiob.pb10);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb10

                    break;
                }
                case 11:
                {
                    switch(af.gpiob.pb11)
                    {
                        case GPIOB_AF_PIN11_TIM2_CH4:
                        case GPIOB_AF_PIN11_I2C2_SDA:
                        case GPIOB_AF_PIN11_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh11 = af.gpiob.pb11;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh11 != af.gpiob.pb11);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb11

                    break;
                }
                case 12:
                {
                    switch(af.gpiob.pb12)
                    {
                        case GPIOB_AF_PIN12_TIM1_BKIN:
                        case GPIOB_AF_PIN12_I2C2_SMBA:
                        case GPIOB_AF_PIN12_SPI2_NSS_I2S2_WS:
                        case GPIOB_AF_PIN12_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh12 = af.gpiob.pb12;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh12 != af.gpiob.pb12);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb12

                    break;
                }
                case 13:
                {
                    switch(af.gpiob.pb13)
                    {
                        case GPIOB_AF_PIN13_TIM1_CH1N:
                        case GPIOB_AF_PIN13_SPI2_SCK_I2S2_CK:
                        case GPIOB_AF_PIN13_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh13 = af.gpiob.pb13;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh13 != af.gpiob.pb13);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb13

                    break;
                }
                case 14:
                {
                    switch(af.gpiob.pb14)
                    {
                        case GPIOB_AF_PIN14_TIM1_CH2N:
                        case GPIOB_AF_PIN14_SPI2_MISO:
                        case GPIOB_AF_PIN14_I2S2ext_SD:
                        case GPIOB_AF_PIN14_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh14 = af.gpiob.pb14;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh14 != af.gpiob.pb14);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb14

                    break;
                }
                case 15:
                {
                    switch(af.gpiob.pb15)
                    {
                        case GPIOB_AF_PIN15_RTC_REFN:
                        case GPIOB_AF_PIN15_TIM1_CH3N:
                        case GPIOB_AF_PIN15_SPI2_MOSI_I2S2_SD:
                        case GPIOB_AF_PIN15_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh15 = af.gpiob.pb15;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh15 != af.gpiob.pb15);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pb5

                    break;
                }
                default:
                {
                    ec = CORE_EC_FAILED;
                    break;
                }
            } // switch pin_no

            break;
        }
        case GPIO_PORT_C:
        {
            switch(pin_no)
            {
                case 0:
                {
                    switch(af.gpioc.pc0)
                    {
                        case GPIOC_AF_PIN0_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl0 = af.gpioc.pc0;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl0 != af.gpioc.pc0);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc0

                    break;
                }
                case 1:
                {
                    switch(af.gpioc.pc1)
                    {
                        case GPIOC_AF_PIN1_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl1 = af.gpioc.pc1;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl1 != af.gpioc.pc1);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc1

                    break;
                }
                case 2:
                {
                    switch(af.gpioc.pc2)
                    {
                        case GPIOC_AF_PIN2_SPI2_MISO:
                        case GPIOC_AF_PIN2_I2S2ext_SD:
                        case GPIOC_AF_PIN2_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl2 = af.gpioc.pc2;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl2 != af.gpioc.pc2);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc2

                    break;
                }
                case 3:
                {
                    switch(af.gpioc.pc3)
                    {
                        case GPIOC_AF_PIN3_SPI2_MOSI_I2S2_SD:
                        case GPIOC_AF_PIN3_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl3 = af.gpioc.pc3;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl3 != af.gpioc.pc3);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc3

                    break;
                }
                case 4:
                {
                    switch(af.gpioc.pc4)
                    {
                        case GPIOC_AF_PIN4_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl4 = af.gpioc.pc4;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl4 != af.gpioc.pc4);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc4

                    break;
                }
                case 5:
                {
                    switch(af.gpioc.pc5)
                    {
                        case GPIOC_AF_PIN5_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl5 = af.gpioc.pc5;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl5 != af.gpioc.pc5);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc5

                    break;
                }
                case 6:
                {
                    switch(af.gpioc.pc6)
                    {
                        case GPIOC_AF_PIN6_TIM3_CH1:
                        case GPIOC_AF_PIN6_I2S2_MCK:
                        case GPIOC_AF_PIN6_USART6_TX:
                        case GPIOC_AF_PIN6_SDIO_D6:
                        case GPIOC_AF_PIN6_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl6 = af.gpioc.pc6;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl6 != af.gpioc.pc6);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc6

                    break;
                }
                case 7:
                {
                    switch(af.gpioc.pc7)
                    {
                        case GPIOC_AF_PIN7_TIM3_CH2:
                        case GPIOC_AF_PIN7_I2S3_MCK:
                        case GPIOC_AF_PIN7_USART6_RX:
                        case GPIOC_AF_PIN7_SDIO_D7:
                        case GPIOC_AF_PIN7_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl7 = af.gpioc.pc7;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl7 != af.gpioc.pc7);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc7

                    break;
                }
                case 8:
                {
                    switch(af.gpioc.pc8)
                    {
                        case GPIOC_AF_PIN8_TIM3_CH3:
                        case GPIOC_AF_PIN8_USART6_CK:
                        case GPIOC_AF_PIN8_SDIO_D0:
                        case GPIOC_AF_PIN8_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh8 = af.gpioc.pc8;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh8 != af.gpioc.pc8);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc8

                    break;
                }
                case 9:
                {
                    switch(af.gpioc.pc9)
                    {
                        case GPIOC_AF_PIN9_MCO_2:
                        case GPIOC_AF_PIN9_TIM3_CH4:
                        case GPIOC_AF_PIN9_I2C3_SDA:
                        case GPIOC_AF_PIN9_I2S2_CKIN:
                        case GPIOC_AF_PIN9_SDIO_D1:
                        case GPIOC_AF_PIN9_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh9 = af.gpioc.pc9;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh9 != af.gpioc.pc9);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc9

                    break;
                }
                case 10:
                {
                    switch(af.gpioc.pc10)
                    {
                        case GPIOC_AF_PIN10_SPI3_SCK_I2S3_CK:
                        case GPIOC_AF_PIN10_SDIO_D2:
                        case GPIOC_AF_PIN10_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh10 = af.gpioc.pc10;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh10 != af.gpioc.pc10);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc10

                    break;
                }
                case 11:
                {
                    switch(af.gpioc.pc11)
                    {
                        case GPIOC_AF_PIN11_I2S3ext_SD:
                        case GPIOC_AF_PIN11_SPI3_MISO:
                        case GPIOC_AF_PIN11_SDIO_D3:
                        case GPIOC_AF_PIN11_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh11 = af.gpioc.pc11;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh11 != af.gpioc.pc11);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc11

                    break;
                }
                case 12:
                {
                    switch(af.gpioc.pc12)
                    {
                        case GPIOC_AF_PIN12_SPI3_MOSI_I2S3_SD:
                        case GPIOC_AF_PIN12_SDIO_CK:
                        case GPIOC_AF_PIN12_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh12 = af.gpioc.pc12;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh12 != af.gpioc.pc12);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc12

                    break;
                }
                case 13:
                {
                    switch(af.gpioc.pc13)
                    {
                        case GPIOC_AF_PIN13_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh13 = af.gpioc.pc13;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh13 != af.gpioc.pc13);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc13

                    break;
                }
                case 14:
                {
                    switch(af.gpioc.pc14)
                    {
                        case GPIOC_AF_PIN14_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh14 = af.gpioc.pc14;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh14 != af.gpioc.pc14);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc14

                    break;
                }
                case 15:
                {
                    switch(af.gpioc.pc15)
                    {
                        case GPIOC_AF_PIN15_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh15 = af.gpioc.pc15;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh15 != af.gpioc.pc15);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pc15

                    break;
                }
                default:
                {
                    ec = CORE_EC_FAILED;
                    break;
                }
            } // switch pin_no

            break;
        }
        case GPIO_PORT_D:
        {
            switch(pin_no)
            {
                case 0:
                {
                    switch(af.gpiod.pd0)
                    {
                        case GPIOD_AF_PIN0_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl0 = af.gpiod.pd0;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl0 != af.gpiod.pd0);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd0

                    break;
                }
                case 1:
                {
                    switch(af.gpiod.pd1)
                    {
                        case GPIOD_AF_PIN1_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl1 = af.gpiod.pd1;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl1 != af.gpiod.pd1);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd1

                    break;
                }
                case 2:
                {
                    switch(af.gpiod.pd2)
                    {
                        case GPIOD_AF_PIN2_TIM3_ETR:
                        case GPIOD_AF_PIN2_SDIO_CMD:
                        case GPIOD_AF_PIN2_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl2 = af.gpiod.pd2;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl2 != af.gpiod.pd2);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd2

                    break;
                }
                case 3:
                {
                    switch(af.gpiod.pd3)
                    {
                        case GPIOD_AF_PIN3_SPI2_SCK_I2S2_CK:
                        case GPIOD_AF_PIN3_USART2_CTS:
                        case GPIOD_AF_PIN3_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl3 = af.gpiod.pd3;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl3 != af.gpiod.pd3);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd3

                    break;
                }
                case 4:
                {
                    switch(af.gpiod.pd4)
                    {
                        case GPIOD_AF_PIN4_USART2_RTS:
                        case GPIOD_AF_PIN4_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl4 = af.gpiod.pd4;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl4 != af.gpiod.pd4);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd4

                    break;
                }
                case 5:
                {
                    switch(af.gpiod.pd5)
                    {
                        case GPIOD_AF_PIN5_USART2_TX:
                        case GPIOD_AF_PIN5_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl5 = af.gpiod.pd5;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl5 != af.gpiod.pd5);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd5

                    break;
                }
                case 6:
                {
                    switch(af.gpiod.pd6)
                    {
                        case GPIOD_AF_PIN6_SPI3_MOSI_I2S3_SD:
                        case GPIOD_AF_PIN6_USART2_RX:
                        case GPIOD_AF_PIN6_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl6 = af.gpiod.pd6;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl6 != af.gpiod.pd6);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd6

                    break;
                }
                case 7:
                {
                    switch(af.gpiod.pd7)
                    {
                        case GPIOD_AF_PIN7_USART2_CK:
                        case GPIOD_AF_PIN7_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl7 = af.gpiod.pd7;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl7 != af.gpiod.pd7);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd7

                    break;
                }
                case 8:
                {
                    switch(af.gpiod.pd8)
                    {
                        case GPIOD_AF_PIN8_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh8 = af.gpiod.pd8;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh8 != af.gpiod.pd8);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd8

                    break;
                }
                case 9:
                {
                    switch(af.gpiod.pd9)
                    {
                        case GPIOD_AF_PIN9_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh9 = af.gpiod.pd9;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh9 != af.gpiod.pd9);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd9

                    break;
                }
                case 10:
                {
                    switch(af.gpiod.pd10)
                    {
                        case GPIOD_AF_PIN10_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh10 = af.gpiod.pd10;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh10 != af.gpiod.pd10);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd10

                    break;
                }
                case 11:
                {
                    switch(af.gpiod.pd11)
                    {
                        case GPIOD_AF_PIN11_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh11 = af.gpiod.pd11;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh11 != af.gpiod.pd11);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd11

                    break;
                }
                case 12:
                {
                    switch(af.gpiod.pd12)
                    {
                        case GPIOD_AF_PIN12_TIM4_CH1:
                        case GPIOD_AF_PIN12_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh12 = af.gpiod.pd12;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh12 != af.gpiod.pd12);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd12

                    break;
                }
                case 13:
                {
                    switch(af.gpiod.pd13)
                    {
                        case GPIOD_AF_PIN13_TIM4_CH2:
                        case GPIOD_AF_PIN13_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh13 = af.gpiod.pd13;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh13 != af.gpiod.pd13);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd13

                    break;
                }
                case 14:
                {
                    switch(af.gpiod.pd14)
                    {
                        case GPIOD_AF_PIN14_TIM4_CH3:
                        case GPIOD_AF_PIN14_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh14 = af.gpiod.pd14;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh14 != af.gpiod.pd14);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd14

                    break;
                }
                case 15:
                {
                    switch(af.gpiod.pd15)
                    {
                        case GPIOD_AF_PIN15_TIM4_CH4:
                        case GPIOD_AF_PIN15_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh15 = af.gpiod.pd15;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh15 != af.gpiod.pd15);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pd15

                    break;
                }
                default:
                {
                    ec = CORE_EC_FAILED;
                    break;
                }
            } // switch pin_no

            break;
        }
        case GPIO_PORT_E:
        {
            switch(pin_no)
            {
                case 0:
                {
                    switch(af.gpioe.pe0)
                    {
                        case GPIOE_AF_PIN0_TIM4_ETR:
                        case GPIOE_AF_PIN0_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl0 = af.gpioe.pe0;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl0 != af.gpioe.pe0);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe0

                    break;
                }
                case 1:
                {
                    switch(af.gpioe.pe1)
                    {
                        case GPIOE_AF_PIN1_TIM1_CH2N:
                        case GPIOE_AF_PIN1_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl1 = af.gpioe.pe1;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl1 != af.gpioe.pe1);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe1

                    break;
                }
                case 2:
                {
                    switch(af.gpioe.pe2)
                    {
                        case GPIOE_AF_PIN2_TRACECLK:
                        case GPIOE_AF_PIN2_SPI4_SCK:
                        case GPIOE_AF_PIN2_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl2 = af.gpioe.pe2;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl2 != af.gpioe.pe2);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe2

                    break;
                }
                case 3:
                {
                    switch(af.gpioe.pe3)
                    {
                        case GPIOE_AF_PIN3_TRACED0:
                        case GPIOE_AF_PIN3_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl3 = af.gpioe.pe3;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl3 != af.gpioe.pe3);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe3

                    break;
                }
                case 4:
                {
                    switch(af.gpioe.pe4)
                    {
                        case GPIOE_AF_PIN4_TRACED1:
                        case GPIOE_AF_PIN4_SPI4_NSS:
                        case GPIOE_AF_PIN4_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl4 = af.gpioe.pe4;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl4 != af.gpioe.pe4);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe4

                    break;
                }
                case 5:
                {
                    switch(af.gpioe.pe5)
                    {
                        case GPIOE_AF_PIN5_TRACED2:
                        case GPIOE_AF_PIN5_TIM9_CH1:
                        case GPIOE_AF_PIN5_SPI4_MISO:
                        case GPIOE_AF_PIN5_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl5 = af.gpioe.pe5;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl5 != af.gpioe.pe5);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe5

                    break;
                }
                case 6:
                {
                    switch(af.gpioe.pe6)
                    {
                        case GPIOE_AF_PIN6_TRACED3:
                        case GPIOE_AF_PIN6_TIM9_CH2:
                        case GPIOE_AF_PIN6_SPI4_MOSI:
                        case GPIOE_AF_PIN6_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl6 = af.gpioe.pe6;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl6 != af.gpioe.pe6);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe6

                    break;
                }
                case 7:
                {
                    switch(af.gpioe.pe7)
                    {
                        case GPIOE_AF_PIN7_TIM1_ETR:
                        case GPIOE_AF_PIN7_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl7 = af.gpioe.pe7;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl7 != af.gpioe.pe7);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe7

                    break;
                }
                case 8:
                {
                    switch(af.gpioe.pe8)
                    {
                        case GPIOE_AF_PIN8_TIM1_CH1N:
                        case GPIOE_AF_PIN8_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh8 = af.gpioe.pe8;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh8 != af.gpioe.pe8);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe8

                    break;
                }
                case 9:
                {
                    switch(af.gpioe.pe9)
                    {
                        case GPIOE_AF_PIN9_TIM1_CH1:
                        case GPIOE_AF_PIN9_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh9 = af.gpioe.pe9;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh9 != af.gpioe.pe9);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe9

                    break;
                }
                case 10:
                {
                    switch(af.gpioe.pe10)
                    {
                        case GPIOE_AF_PIN10_TIM1_CH2N:
                        case GPIOE_AF_PIN10_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh10 = af.gpioe.pe10;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh10 != af.gpioe.pe10);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe10

                    break;
                }
                case 11:
                {
                    switch(af.gpioe.pe11)
                    {
                        case GPIOE_AF_PIN11_TIM1_CH2:
                        case GPIOE_AF_PIN11_SPI4_NSS:
                        case GPIOE_AF_PIN11_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh11 = af.gpioe.pe11;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh11 != af.gpioe.pe11);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe11

                    break;
                }
                case 12:
                {
                    switch(af.gpioe.pe12)
                    {
                        case GPIOE_AF_PIN12_TIM1_CH3N:
                        case GPIOE_AF_PIN12_SPI4_SCK:
                        case GPIOE_AF_PIN12_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh12 = af.gpioe.pe12;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh12 != af.gpioe.pe12);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe12

                    break;
                }
                case 13:
                {
                    switch(af.gpioe.pe13)
                    {
                        case GPIOE_AF_PIN13_TIM1_CH3:
                        case GPIOE_AF_PIN13_SPI4_MISO:
                        case GPIOE_AF_PIN13_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh13 = af.gpioe.pe13;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh13 != af.gpioe.pe13);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe13

                    break;
                }
                case 14:
                {
                    switch(af.gpioe.pe14)
                    {
                        case GPIOE_AF_PIN14_TIM1_CH4:
                        case GPIOE_AF_PIN14_SPI4_MOSI:
                        case GPIOE_AF_PIN14_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh14 = af.gpioe.pe14;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh14 != af.gpioe.pe14);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe14

                    break;
                }
                case 15:
                {
                    switch(af.gpioe.pe15)
                    {
                        case GPIOE_AF_PIN15_TIM1_BKIN:
                        case GPIOE_AF_PIN15_FPU_EVENTOUT:
                        {
                            afrh.bits.afrh15 = af.gpioe.pe15;
                            *afrh_ptr = afrh;
                            do
                            {
                                afrh = *afrh_ptr;
                            }while(afrh.bits.afrh15 != af.gpioe.pe15);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch pe15

                    break;
                }
                default:
                {
                    ec = CORE_EC_FAILED;
                    break;
                }
            } // switch pin_no

            break;
        }
        case GPIO_PORT_H:
        {
            switch(pin_no)
            {
                case 0:
                {
                    switch(af.gpioh.ph0)
                    {
                        case GPIOH_AF_PIN0_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl0 = af.gpioh.ph0;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl0 != af.gpioh.ph0);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch ph0

                    break;
                }
                case 1:
                {
                    switch(af.gpioh.ph1)
                    {
                        case GPIOH_AF_PIN1_FPU_EVENTOUT:
                        {
                            afrl.bits.afrl1 = af.gpioh.ph1;
                            *afrl_ptr = afrl;
                            do
                            {
                                afrl = *afrl_ptr;
                            }while(afrl.bits.afrl1 != af.gpioh.ph1);

                            break;
                        }
                        default:
                        {
                            ec = CORE_EC_FAILED;
                            break;
                        }
                    } // switch ph1

                    break;
                }
                default:
                {
                    ec = CORE_EC_FAILED;
                    break;
                }
            } // switch pin_no

            break;
        }
        default:
        {
            ec = CORE_EC_FAILED;
            break;
        }
    } // switch port

    return ec;
}
