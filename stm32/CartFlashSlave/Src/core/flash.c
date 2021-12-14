/*
 * flash.c
 *
 *  Created on: 15 paź 2021
 *      Author: alf64
 */

#include <core/flash.h>
#include <stm32f401vctx/stm32f401xc.h>

/*
 * ---------- FLASH registers and bits definitions ----------
 */

/*
 * Nomenclature:
 * RO - Read Only
 * RW - Read-Write
 * RC_W1 - Read-Clear-Write-1 (writing 0 has no effect)
 * RC_W0 - Read-Clear-Write-0 (writing 1 has no effect)
 * RS - Read-Set (Writing 0 has no effect)
 */

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t latency : 4; // [RW] sets latency (0 - 15) (0 - no latency, 15 - fifteen cpu cycles wait states)
        uint32_t rsvd_4_7 : 4; // reserved
        uint32_t prften : 1; // [RW] prefetch enable (1) / disable (0)
        uint32_t icen : 1; // [RW] instruction cache enable (1) / disable (0)
        uint32_t dcen : 1; // [RW] data cache enable (1) / disable (0)
        uint32_t icrst : 1; // [RW] instruction cache reset. Can be written only when instruction cache is disabled.
        uint32_t dcrst : 1; // [RW] data cache reset. Can be written only when data cache is disabled.
        uint32_t rsvd_13_31 : 19; // reserved
    }bits;
} flash_reg_acr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Set by hardware when one or more Flash memory operations (program/erase) has/have
        completed successfully. It is set only if the end of operation interrupts are enabled (EOPIE = 1).
        Cleared by writing a 1.
         */
        uint32_t eop : 1; // [RC_W1] - End of Operation.
        /*
         * Set by hardware when a flash operation (programming / erase /read) request is detected and
        can not be run because of parallelism, alignment, or write protection error. This bit is set only if
        error interrupts are enabled (ERRIE = 1)
         */
        uint32_t operr : 1; // [RC_W1] - Operation Error.
        uint32_t rsvd_2_3 : 2; // reserved
        /*
         * Set by hardware when an address to be erased/programmed belongs to a write-protected part
        of the Flash memory.
         */
        uint32_t wrperr : 1; // [RC_W1] Write Protection Error.
        /*
         * Set by hardware when the data to program cannot be contained in the same 128-bit Flash
        memory row.
         */
        uint32_t pgaerr : 1; // [RC_W1] Programming Alignment Error.
        /*
         * Set by hardware when the size of the access (byte, half-word, word, double word) during the
        program sequence does not correspond to the parallelism configuration PSIZE (x8, x16, x32,
        x64).
         */
        uint32_t pgperr : 1; // [RC_W1] Programming Parallelism Error.
        /*
         * Set by hardware when a write access to the Flash memory is performed by the code while the
        control register has not been correctly configured.
         */
        uint32_t pgserr : 1; // [RC_W1] Programming Sequence Error.
        /*
         * Set by hardware when an address to be read through the Dbus belongs to a read protected
        part of the flash.
        Reset by writing 1
         */
        uint32_t rderr : 1; // [RW] Read Protection Error.
        uint32_t rsvd_9_15 : 7; // Reserved.
        /*
         * This bit indicates that a Flash memory operation is in progress. It is set at the beginning of a
        Flash memory operation and cleared when the operation finishes or an error occurs.
        0: no Flash memory operation ongoing
        1: Flash memory operation ongoing
         */
        uint32_t bsy : 1; // [RO] Busy
        uint32_t rsvd_17_31 : 15; // Reserved.

    }bits;
} flash_reg_sr_t;

typedef union
{
    uint32_t value;
    struct
    {
        uint32_t pg : 1; // [RW] Programming. Flash programming activated.
        uint32_t ser : 1; // [RW] Sector Erase. Sector Erase activated.
        uint32_t mer : 1; // [RW] Mass Erase. Erase activated for all user sectors.
        /*
         * These bits select the sector to erase.
        0000 sector 0
        0001 sector 1
        ...
        0101 sector 5
        0110 sector 6 (STM32F401xD/E devices only)
        0111 sector 7 (STM32F401xE devices only)
        1000 not allowed
        ...
        1011 not allowed
        1100 user specific sector
        1101 user configuration sector
        1110 not allowed
        1111 not allowed
         */
        uint32_t snb : 4; // [RW] Sector Number
        uint32_t rsvd8 : 1; // Reserved
        /*
         * These bits select the program parallelism.
        00 program x8
        01 program x16
        10 program x32
        11 program x64
         */
        uint32_t psize : 2; // [RW] Program Size.
        uint32_t rsvd_10_15 : 6; // Reserved.
        /*
         * This bit triggers an erase operation when set. It is set only by software and cleared when the
        BSY bit is cleared.
         */
        uint32_t strt : 1; // [RS] Start.
        uint32_t rsvd_17_23 : 7; // Reserved.
        /*
         * This bit enables the interrupt generation when the EOP bit in the FLASH_SR register goes to 1.
        0: Interrupt generation disabled
        1: Interrupt generation enabled
         */
        uint32_t eopie : 1; // [RW] End of Operation Interrupt Enable.
        /*
         * This bit enables the interrupt generation when the OPERR bit in the FLASH_SR register is set
        to 1.
        0: Error interrupt generation disabled
        1: Error interrupt generation enabled
         */
        uint32_t errie : 1; // [RW] Error Interrupt Enable.
        uint32_t rsvd_26_30 : 5; // Reserved.
        /*
         * Write to 1 only. When it is set, this bit indicates that the FLASH_CR register is locked. It is
        cleared by hardware after detecting the unlock sequence.
        In the event of an unsuccessful unlock operation, this bit remains set until the next reset
         */
        uint32_t lock : 1; // [RS]
    }bits;
} flash_reg_cr_t;

typedef union
{
    uint32_t value;
    struct
    {
        /*
         * Write to 1 only. When this bit is set, it indicates that the FLASH_OPTCR register is locked. This
        bit is cleared by hardware after detecting the unlock sequence.
        In the event of an unsuccessful unlock operation, this bit remains set until the next reset
         */
        uint32_t optlock : 1; // [RS] Option Lock.
        /*
         * This bit triggers a user option operation when set. It is set only by software and cleared when
        the BSY bit is cleared.
         */
        uint32_t optstrt : 1; // [RS] Option start.
        uint32_t rsvd_4 : 1; // Reserved.
        /*
         * These bits contain the supply level threshold that activates/releases the reset. They can be
        written to program a new BOR level. By default, BOR is off. When the supply voltage (VDD)
        drops below the selected BOR level, a device reset is generated.
        00: BOR Level 3 (VBOR3), brownout threshold level 3
        01: BOR Level 2 (VBOR2), brownout threshold level 2
        10: BOR Level 1 (VBOR1), brownout threshold level 1
        11: BOR off, POR/PDR reset threshold level is applied
         */
        uint32_t bor_lev : 2; // [RW] BOR Reset level.
        /*
         * These bits (wdg_sw, nrst_stop, nrst_stdby) contain the value of the user option byte after reset. They can be written to program
         a new user option byte value into Flash memory.
         */
        uint32_t wdg_sw : 1; // [RW] User option byte.
        uint32_t nrst_stop : 1; // [RW] User option byte.
        uint32_t nrst_stdby : 1; // [RW] User option byte.
        /*
         * These bits contain the value of the read-protection option level after reset. They can be written
        to program a new read protection value into Flash memory.
        0xAA: Level 0, read protection not active
        0xCC: Level 2, chip read protection active
        Others: Level 1, read protection of memories active
         */
        uint32_t rdp : 8; // [RW] Read Protect.
        /*
         * These bits contain the value of the write-protection option bytes of sectors after reset. They
        can be written to program a new write protect value into Flash memory.
        0: Write protection active on selected sector
        1: Write protection not active on selected sector
        These bits contain the value of the write-protection and read-protection (PCROP) option
        bytes for sectors 0 to 5 after reset. They can be written to program a new write-protect or
        PCROP value into Flash memory.
        If SPRMOD is reset:
        0: Write protection active on sector i
        1: Write protection not active on sector i
        If SPRMOD is set:
        0: PCROP protection not active on sector i
        1: PCROP protection active on sector i
         */
        uint32_t nwrp : 6; // [RW] Not write protect.
        uint32_t rsvd_22_23 : 2; // Reserved for stm32f401 C/E, however for stm32f401 D/E these are nwrp extended.
        uint32_t rsvd_24_30 : 7; // Reserved.
        /*
         * 0: PCROP disabled, nWPRi bits used for Write Protection on sector i
           1: PCROP enabled, nWPRi bits used for PCROP Protection on sector i
         */
        uint32_t sprmod : 1; // [RW] Selection of protection mode of nwpri bits.
    }bits;
} flash_reg_optcr_t;

typedef struct
{
    /*
     * Flash Access Control Register.
     * Reset value: 0x0.
     * Access: word, half-word, byte access.
     */
    flash_reg_acr_t FLASH_ACR;
    /*
     * Flash Key Register.
     * Reset value: 0x0.
     * Access: word access.
     */
    uint32_t FLASH_KEYR;
    /*
     * Flash Option Key Register.
     * Reset value: 0x0.
     * Access: word access.
     */
    uint32_t FLASH_OPTKEYR;
    /*
     * Flash Status Register.
     * Reset value: 0x0.
     * Access: word, half-word, byte access.
     */
    flash_reg_sr_t FLASH_SR;
    /*
     * Flash Control Register.
     * Reset value: 0x80000000.
     * Access: word, half-word, byte access.
     */
    flash_reg_cr_t FLASH_CR;
    /*
     * Flash Option Control Register.
     * Reset value: 0x0FFFAAED.
     * Access: word, half-word, byte access.
     */
    flash_reg_optcr_t FLASH_OPTCR;
} flash_regs_t;

/*
 * ---------- End of:  FLASH registers and bits definitions ----------
 */


static inline volatile flash_regs_t* flash_get_regs(void)
{
    return (volatile flash_regs_t*)(FLASH_R_BASE);
}

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

void flash_icache_enable(void)
{
    volatile flash_regs_t* flash_regs = flash_get_regs();
    flash_reg_acr_t acr = flash_regs->FLASH_ACR;
    acr.bits.icen = 1;
    flash_regs->FLASH_ACR = acr;
    do
    {
        acr = flash_regs->FLASH_ACR;
    }
    while(!(acr.bits.icen));
}

void flash_icache_disable(void)
{
    volatile flash_regs_t* flash_regs = flash_get_regs();
    flash_reg_acr_t acr = flash_regs->FLASH_ACR;
    acr.bits.icen = 0;
    flash_regs->FLASH_ACR = acr;
    do
    {
        acr = flash_regs->FLASH_ACR;
    }
    while(acr.bits.icen);
}

void flash_dcache_enable(void)
{
    volatile flash_regs_t* flash_regs = flash_get_regs();
    flash_reg_acr_t acr = flash_regs->FLASH_ACR;
    acr.bits.dcen = 1;
    flash_regs->FLASH_ACR = acr;
    do
    {
        acr = flash_regs->FLASH_ACR;
    }
    while(!(acr.bits.dcen));
}

void flash_dcache_disable(void)
{
    volatile flash_regs_t* flash_regs = flash_get_regs();
    flash_reg_acr_t acr = flash_regs->FLASH_ACR;
    acr.bits.dcen = 0;
    flash_regs->FLASH_ACR = acr;
    do
    {
        acr = flash_regs->FLASH_ACR;
    }
    while(acr.bits.dcen);
}

void flash_prefetch_enable(void)
{
    volatile flash_regs_t* flash_regs = flash_get_regs();
    flash_reg_acr_t acr = flash_regs->FLASH_ACR;
    acr.bits.prften = 1;
    flash_regs->FLASH_ACR = acr;
    do
    {
        acr = flash_regs->FLASH_ACR;
    }
    while(!(acr.bits.prften));
}

void flash_prefetch_disable(void)
{
    volatile flash_regs_t* flash_regs = flash_get_regs();
    flash_reg_acr_t acr = flash_regs->FLASH_ACR;
    acr.bits.prften = 0;
    flash_regs->FLASH_ACR = acr;
    do
    {
        acr = flash_regs->FLASH_ACR;
    }
    while(acr.bits.prften);
}

uint8_t flash_get_latency(void)
{
    volatile flash_regs_t* flash_regs = flash_get_regs();
    return (uint8_t)(flash_regs->FLASH_ACR.bits.latency);
}

core_ec_t flash_set_latency(uint8_t latency)
{
    if(latency > 5)
    {
        return CORE_EC_FAILED;
    }

    volatile flash_regs_t* flash_regs = flash_get_regs();
    flash_reg_acr_t acr = flash_regs->FLASH_ACR;
    acr.bits.latency = (uint8_t)(latency & 0xF);
    flash_regs->FLASH_ACR = acr;
    do
    {
        acr = flash_regs->FLASH_ACR;
    }while(acr.bits.latency != latency);

    return CORE_EC_SUCCESS;
}
