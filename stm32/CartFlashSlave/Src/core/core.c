/*
 * core.c
 *
 *  Created on: 25 paź 2021
 *      Author: alf64
 */

#include <core/core.h>


/* -------------- ARMv7-M core specific defines -------------- */
/*
 * vectkey value needed for altering the AIRCR register.
 * You need to write it under vectkey bit field whenever you perform write to AIRCR,
 * otherwise write will be ignored.
 */
#define CORE_ARM_VECTKEY_WRITE_VALUE 0x05FA

/*
 * vectkey (vectkeystat) value read from the AIRCR register.
 * This value is always returned from the vectkey (vectkeystat) field of the AIRCR
 * register whenever read is performed.
 */
#define CORE_ARM_VECTKEY_READ_VALUE 0xFA05

/* -------------- ARMv7-M core specific registers definitions -------------- */
/*
 * @brief Bit definitions for Application Interrupt and Reset Control Register (AIRCR).
 * This register is located in the System Control Block (SCB).
 */
typedef union
{
    uint32_t value;
    struct
    {
        uint32_t vectreset : 1; // Writing '1' to this bit causes Local System Reset. Write-only. Self clears.
        uint32_t vectclractive : 1; // Writing '1' to this bit clears all active state information for exceptions, clears IPSR. Write-only.
        uint32_t sysresetreq : 1; // '1' - request Local reset (asserts a signal). Local reset or power-on clears this bit.
        uint32_t rsvd3_7 : 5; // Reserved
        uint32_t prigroup : 3; // sets priority grouping scheme
        uint32_t rsvd11_14 : 4; // Reserved
        uint32_t endianess : 1; // Let's you know about core endianness. '0' - little endian, '1' - big endian.
        uint32_t vectkey : 16;
    }bits;
}core_arm_reg_aircr_t;

/*
 * @brief Bit definitions for Interrupt Controller Type Register (ICTR).
 * This register is located in the System Control Space (SCS).
 */
typedef union
{
    uint32_t value;
    struct
    {
        /*
         * The total number of interrupt lines supported by an implementation, defined in groups of
        32. That is, the total number of interrupt lines is up to (32*(INTLINESNUM+1)). However,
        the absolute maximum number of interrupts is 496, corresponding to the INTLINESNUM
        value 0b1111.
        INTLINESNUM indicates which registers in the NVIC register map are implemented.
        Example: if intlinesnum == 0, then total number of interrupt lines = 32.
         */
        uint32_t intlinesnum : 4;
        uint32_t rsvd4_31 : 28;
    }bits;
}core_arm_reg_ictr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Indicates the enabled status of the SysTick counter:
        0 Counter is disabled.
        1 Counter is operating
         */
        uint32_t enable : 1; // [RW] Systick timer/counter enable.
        /*
         * Indicates whether counting to 0 causes the status of the SysTick exception to change to
        pending:
        0 Count to 0 does not affect the SysTick exception status.
        1 Count to 0 changes the SysTick exception status to pending.
        Worth noting: Changing the value of the counter to 0 by writing zero to the SysTick Current Value register
        to 0 never changes the status of the SysTick exception
         */
        uint32_t tickint : 1; // [RW] Enables an interrupt when timer/counter reaches 0.
        /*
         * Indicates the SysTick clock source:
        0 SysTick uses the IMPLEMENTATION DEFINED external reference clock.
        1 SysTick uses the processor clock (typical setting).
        If no external clock is provided, this bit reads as 1 and ignores writes
         */
        uint32_t clksource : 1; // [RW] Clock source for the Systick timer/counter.
        uint32_t rsvd3_15 : 13; // reserved
        /*
         * Indicates whether the counter has counted to 0 since the last read of this register:
        0 Timer has not counted to 0.
        1 Timer has counted to 0.
        COUNTFLAG is set to 1 by a count transition from 1 to 0.
        COUNTFLAG is cleared to 0 by a software read of this register, and by any write to the
        Current Value register. Debugger reads do not clear the COUNTFLAG.
        This bit is read-only.
         */
        uint32_t countflag : 1; // [RO] Counter transition from 1 to 0 flag.
        uint32_t rsvd17_31 : 15; // reserved
    }bits;
}core_arm_reg_syst_csr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t reload : 24; // [RW] The value to load into the SYST_CVR register when the counter reaches 0.
        uint32_t rsvd224_31 : 8; // reserved
    }bits;
}core_arm_reg_syst_rvr_t;

/*
 *  Current counter value. This is the value of the counter at the time it is sampled.
 */
typedef uint32_t core_arm_reg_syst_cvr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Optionally, holds a reload value to be used for 10ms (100Hz) timing, subject to system clock
        skew errors. If this field is zero, the calibration value is not known.
         */
        uint32_t tenms : 24; // [RO] Ten Miliseconds.
        uint32_t rsvd24_29 : 6; // reserved
        /*
         * Indicates whether the 10ms calibration value is exact:
        0 10ms calibration value is exact.
        1 10ms calibration value is inexact, because of the clock frequency
         */
        uint32_t skew : 1; // [RO] Skew.
        /*
         * Indicates whether the IMPLEMENTATION DEFINED reference clock is implemented:
        0 The reference clock is implemented.
        1 The reference clock is not implemented.
        When this bit is 1, the CLKSOURCE bit of the SYST_CSR register is forced to 1 and cannot
        be cleared to 0.
         */
        uint32_t noref : 1; // [RO] No IMPLEMENTATION DEFINED reference clock implemented.
    }bits;
}core_arm_reg_syst_calib_t;

typedef struct
{
    core_arm_reg_syst_csr_t SYST_CSR; // Systick Control and Status Register
    core_arm_reg_syst_rvr_t SYST_RVR; // Systick Reload Value Register
    core_arm_reg_syst_cvr_t SYST_CVR; // Systick Current Value Register
    core_arm_reg_syst_calib_t SYST_CALIB; //
}core_arm_syst_regs_t;

/* -------------- End of ARMv7-M core specific registers definitions -------------- */

/*
 * Returns pointer to SYST (SysTick) registers.
 */
static inline volatile core_arm_syst_regs_t* core_get_syst_regs(void)
{
    return (volatile core_arm_syst_regs_t*)(SysTick_BASE);
}

core_ec_t core_nvic_set_priogrp(core_nvic_priogrp_t priority_grouping)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    switch(priority_grouping)
    {
        case CORE_NVIC_PRIOGRP_4_0:
        case CORE_NVIC_PRIOGRP_3_1:
        case CORE_NVIC_PRIOGRP_2_2:
        case CORE_NVIC_PRIOGRP_1_3:
        case CORE_NVIC_PRIOGRP_0_4:
        {
            // You can use the function provided by core_cm4.h...
            // NVIC_SetPriorityGrouping(priority_grouping);

            // ... but let's do this our way!
            core_arm_reg_aircr_t aircr = (core_arm_reg_aircr_t)(SCB->AIRCR);
            if(aircr.bits.vectkey != CORE_ARM_VECTKEY_READ_VALUE)
            {
                // seems like we did not hit the AIRCR register
                ec = CORE_EC_FAILED;
                break;
            }
            aircr.bits.vectkey = CORE_ARM_VECTKEY_WRITE_VALUE;
            // need only 3 lower bits since prigroup field is 3-bit wide
            uint8_t prigroup = (uint8_t)(priority_grouping & 0x7);
            aircr.bits.prigroup = (unsigned char)(prigroup & 0x7);
            SCB->AIRCR = aircr.value;
            break;
        }
        case CORE_NVIC_PRIOGRP_RSVD1:
        case CORE_NVIC_PRIOGRP_RSVD2:
        case CORE_NVIC_PRIOGRP_RSVD3:
        default:
        {
            ec = CORE_EC_FAILED;
            break;
        }
    }

    return ec;
}

core_ec_t core_nvic_set_priority(IRQn_Type irq, uint32_t preeprio, uint32_t subprio)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    // check if we got irq that makes sense (fits within maximum interrupt or exception lines)
    if(irq >= 0)
    { // interrupt specific to STM32
        uint32_t ilinesnum = ((core_arm_reg_ictr_t)(SCnSCB->ICTR)).bits.intlinesnum;
        if(((uint32_t)(irq)) > (ilinesnum-1))
        {
            ec = CORE_EC_FAILED;
        }
    }
    else
    { // interrupt specific to ARM core
        switch(irq)
        {
            // ARM IRQs
            case NonMaskableInt_IRQn:
            case MemoryManagement_IRQn:
            case BusFault_IRQn:
            case UsageFault_IRQn:
            case SVCall_IRQn:
            case DebugMonitor_IRQn:
            case PendSV_IRQn:
            case SysTick_IRQn:
            {
                break;
            }
            // STM32 IRQs
            case WWDG_IRQn:
            case PVD_IRQn:
            case TAMP_STAMP_IRQn:
            case RTC_WKUP_IRQn:
            case FLASH_IRQn:
            case RCC_IRQn:
            case EXTI0_IRQn:
            case EXTI1_IRQn:
            case EXTI2_IRQn:
            case EXTI3_IRQn:
            case EXTI4_IRQn:
            case DMA1_Stream0_IRQn:
            case DMA1_Stream1_IRQn:
            case DMA1_Stream2_IRQn:
            case DMA1_Stream3_IRQn:
            case DMA1_Stream4_IRQn:
            case DMA1_Stream5_IRQn:
            case DMA1_Stream6_IRQn:
            case ADC_IRQn:
            case EXTI9_5_IRQn:
            case TIM1_BRK_TIM9_IRQn:
            case TIM1_UP_TIM10_IRQn:
            case TIM1_TRG_COM_TIM11_IRQn:
            case TIM1_CC_IRQn:
            case TIM2_IRQn:
            case TIM3_IRQn:
            case TIM4_IRQn:
            case I2C1_EV_IRQn:
            case I2C1_ER_IRQn:
            case I2C2_EV_IRQn:
            case I2C2_ER_IRQn:
            case SPI1_IRQn:
            case SPI2_IRQn:
            case USART1_IRQn:
            case USART2_IRQn:
            case EXTI15_10_IRQn:
            case RTC_Alarm_IRQn:
            case OTG_FS_WKUP_IRQn:
            case DMA1_Stream7_IRQn:
            case SDIO_IRQn:
            case TIM5_IRQn:
            case SPI3_IRQn:
            case DMA2_Stream0_IRQn:
            case DMA2_Stream1_IRQn:
            case DMA2_Stream2_IRQn:
            case DMA2_Stream3_IRQn:
            case DMA2_Stream4_IRQn:
            case OTG_FS_IRQn:
            case DMA2_Stream5_IRQn:
            case DMA2_Stream6_IRQn:
            case DMA2_Stream7_IRQn:
            case USART6_IRQn:
            case I2C3_EV_IRQn:
            case I2C3_ER_IRQn:
            case FPU_IRQn:
            case SPI4_IRQn:
            default:
            {
                ec = CORE_EC_FAILED;
                break;
            }
        }
    }

    if(ec != CORE_EC_SUCCESS)
    {
        return ec;
    }

    /*
     * get the current priority grouping configuration
     * in general you could use:
     * __NVIC_GetPriorityGrouping(void) from core_cm4.h, but let's do this our way!
     */
    core_arm_reg_aircr_t aircr = (core_arm_reg_aircr_t)(SCB->AIRCR);
    uint8_t prigroup = aircr.bits.prigroup;

    /*
     * now encode how much bits are used for preemption priority and subpriority
     * in general you could use:
     * NVIC_EncodePriority() from core_cm4.h, but let's do this our way!
     */
    uint8_t preeprio_bits = 0;
    uint8_t subprio_bits = 0;
    uint16_t preeprio_max_value = 0;
    uint16_t subprio_max_value = 0;
    switch(prigroup)
    {
        case CORE_NVIC_PRIOGRP_4_0:
        {
            preeprio_bits = 4;
            subprio_bits = 0;

            if((preeprio > 15) || (subprio > 0))
            {
                ec = CORE_EC_FAILED;
            }

            break;
        }
        case CORE_NVIC_PRIOGRP_3_1:
        {
            preeprio_bits = 3;
            subprio_bits = 1;

            if((preeprio > 7) || (subprio > 1))
            {
                ec = CORE_EC_FAILED;
            }

            break;
        }
        case CORE_NVIC_PRIOGRP_2_2:
        {
            preeprio_bits = 2;
            subprio_bits = 2;

            if((preeprio > 4) || (subprio > 4))
            {
                ec = CORE_EC_FAILED;
            }

            break;
        }
        case CORE_NVIC_PRIOGRP_1_3:
        {
            preeprio_bits = 1;
            subprio_bits = 3;

            if((preeprio > 1) || (subprio > 7))
            {
                ec = CORE_EC_FAILED;
            }

            break;
        }
        case CORE_NVIC_PRIOGRP_0_4:
        {
            preeprio_bits = 0;
            subprio_bits = 4;

            if((preeprio > 0) || (subprio > 15))
            {
                ec = CORE_EC_FAILED;
            }

            break;
        }
        case CORE_NVIC_PRIOGRP_RSVD1:
        case CORE_NVIC_PRIOGRP_RSVD2:
        case CORE_NVIC_PRIOGRP_RSVD3:
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
    else
    {
        preeprio_max_value = (uint16_t)(((uint16_t)(1<<preeprio_bits)) - 1);
        subprio_max_value = (uint16_t)(((uint16_t)(1<<subprio_bits)) - 1);

        if((preeprio > preeprio_max_value) || (subprio > subprio_max_value))
        {
            return CORE_EC_FAILED;
        }
    }

    /*
     * The result of the encoding is a variable
     * that consists of concatenated preeprio and subprio,
     * preeprio being more significant than subprio
     */
    uint8_t priority =
            (uint8_t)(((preeprio & preeprio_max_value) << subprio_bits) |
            (subprio & subprio_max_value));

    /*
     * now set the priority
     * in general you could use:
     * NVIC_SetPriority() from core_cm4.h, but let's do this our way!
     * IRQn_Type >= 0 are device specific interrupts, for which priority is set
     * via IPR registers within ARM NVIC register area
     * IRQn_Type < 0 are ARM core specific interrupts (called "exceptions"), for which priority
     * is set via SHP register within ARM SCB (System Control Block) register area.
     */
    if(irq >= 0)
    {
        NVIC->IP[((uint32_t)irq)] = (uint8_t)((priority << ((uint8_t)8 - __NVIC_PRIO_BITS)) & 0xFF);
    }
    else
    {
        /*
         * Now here comes the tricky part!
         * From the ARMv7-m manual, ARM exceptions numbers are ordered as follows:
         * 1 = Reset (the highest priority)
         * 2 = NMI
         * 3 = HardFault
         * 4 = MemManage
         * 5 = BusFault
         * 6 = UsageFault
         * 7 - 10 = Reserved
         * 11 = SVCall
         * 12 = DebugMonitor
         * 13 = Reserved
         * 14 = PendSV
         * 15 = SysTick
         * ----- From now on, the device interrupts starts (called "external interrupts", as from the
         * ARM core view they are external) ------------------------
         * 16 = External Interrupt 0 (for the STM32F401, it's Window Watchdog Interrupt)
         * 17 = External Interrupt 1 (for the STM32F401, it's PVD through EXTI line detection interrupt)
         * ... and so on...
         * @see g_pfnVectors in the startup_stm32f401vctx.s, as they are ordered in this way there.
         *
         * Now if you take a look into stm32f401xc.h, you will find that IRQn_Type defines the
         * numbers in a way that External Interrupt 0 (watchdog) is 0, and ARM exceptions are
         * negative numbers before it.
         *
         * This is a little different approach, for easy conversion to get the proper entry in the SHPR registers ;)
         * Also, External Interrupts begins from 0, which is a convenient way when working mostly with them and
         * when programming IPR registers (see above in this 'if' statement).
         *
         * To access the correct ARM exception priority field in the SHPR register, on the negative IRQn_Type
         * the cast from int to uint is performed and 0xF mask is applied
         * (since ARM exceptions are numbered from 1 to 15, so only 4 bits are used).
         *
         * Example:
         * IRQn_Type: NonMaskableInt_IRQn = -14
         * The binary representative (in a int8 type) for this int is: 0b11110010
         * Now if you cast it to uint, you still have 0b11110010 but it's no longer interpreted as -14,
         * but it's now 242 (if we still operate on int8 type, now being uint8).
         * If you add a mask of 0xF by performing &0xF on this, you get 0b00000010, which is now 2.
         * 2 is a number of the NMI exception in the ARM exceptions order :)
         *
         * Another important things is that the first 3 exceptions (Reset, NMI and HardFault) are the
         * most important and their priority cannot be changed. There is no place in SHPR registers
         * to change their priority, meaning you have to exclude them when programming SHPR registers
         * (the first priority in SHPR is the priority for MemManage exception).
         * Also, exceptions are numbered from 1, while SHPR register is indexed from 0, meaning
         * that in fact you have to exclude value of 4.
         */

        uint8_t exc_shpr_index = 0;
        exc_shpr_index = (uint8_t)(((uint32_t)irq) & 0xF);
        // exclude 4 things: Reset, NMI, HardFault and the fact the exceptions are numbered from 1
        exc_shpr_index = (uint8_t)(exc_shpr_index - ((uint8_t)4));
        SCB->SHP[exc_shpr_index] = (uint8_t)((priority << ((uint8_t)8 - __NVIC_PRIO_BITS)) & 0xFF);
    }

    return ec;
}


core_ec_t core_systick_config_and_enable(uint32_t interval, uint8_t preeprio, uint8_t subprio)
{
    core_ec_t ec = CORE_EC_SUCCESS;

    /*
     * Interval is decremented by 1, since
     * Current Value Register (CVR) is being reloaded with Reload Value Register (RVR)
     * on the next clock tick upon reaching zero. This means there is 1 tick overhead of
     * what interval we want.
     */
    uint32_t rvr_value = interval - 1;

    // check if rvr_value is within 24bit range
    if(rvr_value > SysTick_LOAD_RELOAD_Msk)
    {
        return CORE_EC_FAILED;
    }

    volatile core_arm_syst_regs_t* syst_regs = core_get_syst_regs();
    core_arm_reg_syst_csr_t csr = syst_regs->SYST_CSR;
    core_arm_reg_syst_rvr_t rvr = syst_regs->SYST_RVR;

    // set reload value
    rvr.bits.reload = (uint32_t)(rvr_value & 0xFFF);
    syst_regs->SYST_RVR = rvr;
    do
    {
        rvr = syst_regs->SYST_RVR;
    }
    while(rvr.bits.reload != rvr_value);

    // set systick interrupt priority to the highest
    ec = core_nvic_set_priority(SysTick_IRQn, preeprio, subprio);
    if(ec != CORE_EC_SUCCESS)
    {
        return ec;
    }

    // reset the systick counter to zero
    syst_regs->SYST_CVR = 0;

    // set things up and enable
    csr.bits.clksource = 1; // use core clock (HCLK) for systick counter
    csr.bits.tickint = 1; // enable systick interrupt
    csr.bits.enable = 1; // enable systick timer/counter
    syst_regs->SYST_CSR = csr;
    do
    {
        csr = syst_regs->SYST_CSR;
    }
    while((csr.bits.clksource != 1) && (csr.bits.tickint != 1) && (csr.bits.enable != 1));

    return ec;
}
