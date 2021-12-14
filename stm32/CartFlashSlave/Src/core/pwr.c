/*
 * pwr.c
 *
 *  Created on: 27 Oct 2021
 *      Author: alf64
 */

#include <core/pwr.h>
#include <core/rcc.h>
#include <stm32f401vctx/stm32f401xc.h>

/*
 * ---------- PWR registers and bits definitions ----------
 */

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

typedef union
{
    uint32_t value;
    struct
    {
        /*
        This bit is set and cleared by software. It works together with the PDDS bit.
        0: Voltage regulator on during Stop mode.
        1: Low-power Voltage regulator on during Stop mode.
         */
        uint32_t lpds : 1; // [RW] Low-power deepsleep.
        /*
        This bit is set and cleared by software. It works together with the LPDS bit.
        0: Enter Stop mode when the CPU enters deepsleep. The regulator status depends on the
        LPDS bit.
        1: Enter Standby mode when the CPU enters deepsleep.
         */
        uint32_t pdds : 1; // [RW] Power-down deepsleep.
        /*
        This bit is always read as 0.
        0: No effect.
        1: Clear the WUF Wakeup Flag after 2 System clock cycles.
         */
        uint32_t cwuf : 1; // [W] Clear Wakeup Flag.
        /*
        This bit is always read as 0.
        0: No effect.
        1: Clear the SBF Standby Flag (write)
         */
        uint32_t csbf : 1; // [W] Clear StandBy Flag.
        /*
        This bit is set and cleared by software.
        0: PVD disabled
        1: PVD enabled
         */
        uint32_t pvde : 1; // [RW] Power Voltage Detector Enable.
        /*
        These bits are written by software to select the voltage threshold detected by the Power
        Voltage Detector
        000: 2.2 V
        001: 2.3 V
        010: 2.4 V
        011: 2.5 V
        100: 2.6 V
        101: 2.7 V
        110: 2.8 V
        111: 2.9 V
         */
        uint32_t pls : 3; // [RW] Power Voltage Detector (PVD) level selection
        /*
        In reset state, the RCC_BDCR register, the RTC registers (including the backup registers),
        and the BRE bit of the PWR_CSR register, are protected against parasitic write access. This
        bit must be set to enable write access to these registers.
        0: Access to RTC and RTC Backup registers.
        1: Access to RTC and RTC Backup registers
         */
        uint32_t dbp : 1; // [RW] Disable backup Domain Write Protection.
        /*
        When set, the Flash memory enters power-down mode when the device enters Stop mode.
        This allows to achieve a lower consumption in stop mode but a longer restart time.
        0: Flash memory not in power-down when the device is in Stop mode
        1: Flash memory in power-down when the device is in Stop mode
         */
        uint32_t fpds : 1; // [RW] Flash Power-Down in Stop mode.
        /*
        0: Low-power regulator on if LPDS bit is set when the device is in Stop mode.
        1: Low-power regulator in Low Voltage and Flash memory in Deep Sleep mode if LPDS bit is
        set when device is in Stop mode.
         */
        uint32_t lplvds : 1; // [RW] Low-power regulator Low Voltage in Deep Sleep.
        /*
        0: Main regulator in Voltage scale 3 when the device is in Stop mode.
        1: Main regulator in Low Voltage and Flash memory in Deep Sleep mode when the device is
        in Stop mode.
         */
        uint32_t mrlvds : 1; // [RW] Main regulator Low Voltage in Deep Sleep.
        uint32_t rsvd12 : 1; // Reserved
        /*
        0: No effect.
        1: Refer to AN4073 for details on how to use this bit.
         */
        uint32_t adcdc1 : 1; // [RW] ?? Refer to AN4073 on how to use this bit.
        /*
        These bits control the main internal voltage regulator output voltage to achieve a trade-off
        between performance and power consumption when the device does not operate at the
        maximum frequency (refer to the corresponding datasheet for more details).
        These bits can be modified only when the PLL is OFF. The new value programmed is active
        only when the PLL is ON. When the PLL is OFF, the voltage regulator is set to scale 3
        independently of the VOS register content.
        00: Reserved (Scale 3 mode selected)
        01: Scale 3 mode
        10: Scale 2 mode
        11: Reserved (Scale 2 mode selected)
         */
        uint32_t vos : 2; // [RW] Regulator voltage scaling output selection.
        uint32_t rsvd16_31 : 16; // Reserved
    }bits;
}pwr_reg_cr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
        This bit is set by hardware and cleared either by a system reset or by setting the CWUF bit in
        the PWR_CR register.
        0: No wakeup event occurred
        1: A wakeup event was received from the WKUP pin or from the RTC alarm (Alarm A or
        Alarm B), RTC Tamper event, RTC TimeStamp event or RTC Wakeup).
        Note: An additional wakeup event is detected if the WKUP pin is enabled (by setting the
        EWUP bit) when the WKUP pin level is already high.
         */
        uint32_t wuf : 1; // [RO] WakeUp Flag.
        /*
         This bit is set by hardware and cleared only by a POR/PDR (power-on reset/power-down
        reset) or by setting the CSBF bit in the PWR_CR register.
        0: Device has not been in Standby mode
        1: Device has been in Standby mode
         */
        uint32_t sbf : 1; // [RO] StandBy Flag.
        /*
        This bit is set and cleared by hardware. It is valid only if PVD is enabled by the PVDE bit.
        0: VDD is higher than the PVD threshold selected with the PLS[2:0] bits.
        1: VDD is lower than the PVD threshold selected with the PLS[2:0] bits.
        Note: The PVD is stopped by Standby mode. For this reason, this bit is equal to 0 after
        Standby or reset until the PVDE bit is set.
         */
        uint32_t pvdo : 1; // [RO] PVD Output.
        /*
        Set by hardware to indicate that the Backup Regulator is ready.
        0: Backup Regulator not ready
        1: Backup Regulator ready
        Note: This bit is not reset when the device wakes up from Standby mode or by a system reset
        or power reset.
         */
        uint32_t brr : 1; // [RO] Backup Regulator Ready.
        uint32_t rsvd4_7 : 4; // Reserved
        /*
        This bit is set and cleared by software.
        0: WKUP pin is used for general purpose I/O. An event on the WKUP pin does not wakeup
        the device from Standby mode.
        1: WKUP pin is used for wakeup from Standby mode and forced in input pull down
        configuration (rising edge on WKUP pin wakes-up the system from Standby mode).
        Note: This bit is reset by a system reset.
         */
        uint32_t ewup : 1; // [RW] Enable WKUP pin.
        /*
        When set, the Backup regulator (used to maintain the backup domain content) is enabled. If
        BRE is reset, the backup regulator is switched off. Once set, the application must wait that
        the Backup Regulator Ready flag (BRR) is set to indicate that the data written into the
        backup registers will be maintained in the Standby and VBAT modes.
        0: Backup regulator disabled
        1: Backup regulator enabled
        Note: This bit is not reset when the device wakes up from Standby mode, by a system reset,
        or by a power reset.
         */
        uint32_t bre : 1; // [RW] Backup Regulator Enable.
        uint32_t rsvd10_13 : 4; // Reserved
        uint32_t vosrdy : 1; // [RO] Regulator voltage scaling output selection ready bit. 0 - Not ready, 1 - Ready.
        uint32_t rsvd15_31 : 17; // Reserved
    }bits;
}pwr_reg_csr_t;

typedef struct
{
    /*
     * Power (PWR) Control Register (CR)
    Address offset: 0x00
    Reset value: 0x0000 8000 (reset by wakeup from Standby mode)
     */
    pwr_reg_cr_t PWR_CR;
    /*
     * Power (PWR) Control/Status Register (CSR)
    Address offset: 0x04
    Reset value: 0x0000 0000 (not reset by wakeup from Standby mode)
    Additional APB cycles are needed to read this register versus a standard APB read
     */
    pwr_reg_csr_t PWR_CSR;
}pwr_regs_t;

/*
 * ---------- End of:  PWRregisters and bits definitions ----------
 */


static inline volatile pwr_regs_t* pwr_get_regs(void)
{
    return (volatile pwr_regs_t*)(PWR_BASE);
}

core_ec_t pwr_set_voltage_output_scaling(pwr_vos_t vos)
{
    if(rcc_is_main_pll_on())
    {
        // can't change VOS when PLL is active
        return CORE_EC_FAILED;
    }

    switch(vos)
    {
        case PWR_VOS_SCALE2_MODE:
        case PWR_VOS_SCALE3_MODE:
        {
            // valid options
            break;
        }
        case PWR_VOS_RESERVED1:
        case PWR_VOS_RESERVED2:
        default:
        {
            // invalid option selected
            return CORE_EC_FAILED;
        }
    }

    volatile pwr_regs_t* pwr_regs = pwr_get_regs();
    pwr_reg_cr_t cr = pwr_regs->PWR_CR;
    cr.bits.vos = vos;
    pwr_regs->PWR_CR = cr;
    do
    {
        cr = pwr_regs->PWR_CR;
    }
    while(cr.bits.vos != vos);

    return CORE_EC_SUCCESS;
}

