/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <RTL.h>
#include "target_config.h"
#include "target_reset.h"
#include "jtag_host.h"
#include "debug_cm.h"
#include "DAP_config.h"
#include "DAP.h"

#if DAP_JTAG

// Default NVIC and Core debug base addresses
// TODO: Read these addresses from ROM.
#define NVIC_Addr    (0xe000e000)
#define DBG_Addr     (0xe000edf0)

// AP CSW register, base value
#define CSW_VALUE (CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC)

// SWD register access
#define REG_AP      (1)
#define REG_DP      (0)
#define REG_R       (1<<1)
#define REG_W       (0<<1)
#define REG_ADR(a)  (a & 0x0c)

#define DCRDR 0xE000EDF8
#define DCRSR 0xE000EDF4
#define DHCSR 0xE000EDF0
#define REGWnR (1 << 16)

#define MAX_RETRY 10
#define MAX_TIMEOUT   10000  // Timeout for syscalls on target

// Some targets require a soft reset for flash programming (RESET_PROGRAM).
// Otherwise a hardware reset is the default. This will not affect
// DAP operations as they are controlled by the remote debugger.
#if defined(BOARD_BAMBINO_210) || defined(BOARD_BAMBINO_210E)
#define CONF_SYSRESETREQ
#elif defined(BOARD_LPC4337)
#define CONF_VECTRESET
#endif

#if defined(CONF_SYSRESETREQ)
// SYSRESETREQ - Software reset of the Cortex-M core and on-chip peripherals
#define SOFT_RESET  SYSRESETREQ

#elif defined(CONF_VECTRESET)
// VECTRESET - Software reset of Cortex-M core
// For some Cortex-M devices, VECTRESET is the only way to reset the core.
// VECTRESET is not supported on Cortex-M0 and Cortex-M1 cores.
#define SOFT_RESET  VECTRESET

#endif

typedef struct {
    uint32_t select;
    uint32_t csw;
} DAP_STATE;

typedef struct {
    uint32_t r[16];
    uint32_t xpsr;
} DEBUG_STATE;

static DAP_STATE dap_state;

static uint8_t jtag_read_core_register(uint32_t n, uint32_t *val);
static uint8_t jtag_write_core_register(uint32_t n, uint32_t val);

static void int2array(uint8_t * res, uint32_t data, uint8_t len) {
    uint8_t i = 0;
    for (i = 0; i < len; i++) {
        res[i] = (data >> 8*i) & 0xff;
    }
}

static uint8_t jtag_transfer_retry(uint32_t req, uint32_t * data) {
    uint8_t i, ack;
    for (i = 0; i < MAX_RETRY; i++) {
        ack = JTAG_Transfer(req, data);
        // if ack != WAIT
        if (ack != 0x02) {
            return ack;
        }
    }
    return ack;
}


uint8_t jtag_init(void) {
    DAP_Setup();
    PORT_JTAG_SETUP();
    return 1;
}

uint8_t jtag_off(void)
{
    PORT_OFF();
    return 1;
}

// Read debug port register.
uint8_t jtag_read_dp(uint8_t adr, uint32_t *val) {
    uint32_t tmp_in;
    uint8_t tmp_out[4];
    uint8_t ack;

    tmp_in = REG_DP | REG_R | REG_ADR(adr);

    // first dummy read
    JTAG_IR(JTAG_DPACC);
    jtag_transfer_retry(tmp_in, (uint32_t *)tmp_out);

    tmp_in = REG_DP | REG_R | REG_ADR(DP_RDBUFF);
    ack = jtag_transfer_retry(tmp_in, (uint32_t *)tmp_out);

    *val = (tmp_out[3] << 24) | (tmp_out[2] << 16) | (tmp_out[1] << 8) | tmp_out[0];

    return (ack == 0x01);
}

// Write debug port register
uint8_t jtag_write_dp(uint8_t adr, uint32_t val) {
    uint32_t req;
    uint8_t data[4];
    uint8_t ack;

    if (adr == DP_SELECT) {
       if (dap_state.select == val)
            return 1;
        dap_state.select = val;
    }

    req = REG_DP | REG_W | REG_ADR(adr);
    int2array(data, val, 4);

    JTAG_IR(JTAG_DPACC);
    ack = jtag_transfer_retry(req, (uint32_t *)data);

    return (ack == 0x01);
}

// Read access port register.
uint8_t jtag_read_ap(uint32_t adr, uint32_t *val) {
    uint8_t tmp_in, ack;
    uint8_t tmp_out[4];

    uint32_t apsel = adr & 0xff000000;
    uint32_t bank_sel = adr & APBANKSEL;

    if (!jtag_write_dp(DP_SELECT, apsel | bank_sel)) {
        return 0;
    }

    JTAG_IR(JTAG_APACC);

    tmp_in = REG_AP | REG_R | REG_ADR(adr);

    // first dummy read
    jtag_transfer_retry(tmp_in, (uint32_t *)tmp_out);
    ack = jtag_transfer_retry(tmp_in, (uint32_t *)tmp_out);
    *val = (tmp_out[3] << 24) | (tmp_out[2] << 16) | (tmp_out[1] << 8) | tmp_out[0];

    return (ack == 0x01);
}

// Write access port register
uint8_t jtag_write_ap(uint32_t adr, uint32_t val) {
    uint8_t data[4];
    uint8_t req, ack;
    uint32_t apsel = adr & 0xff000000;
    uint32_t bank_sel = adr & APBANKSEL;

    if (!jtag_write_dp(DP_SELECT, apsel | bank_sel)) {
        return 0;
    }

    if (adr == AP_CSW) {
        if (dap_state.csw == val)
            return 1;
        dap_state.csw = val;
    }

    JTAG_IR(JTAG_APACC);
    req = REG_AP | REG_W | REG_ADR(adr);
    int2array(data, val, 4);
    if (jtag_transfer_retry(req, (uint32_t *)data) != 0x01) {
        return 0;
    }

    JTAG_IR(JTAG_DPACC);
    req = REG_DP | REG_R | REG_ADR(DP_RDBUFF);
    ack = jtag_transfer_retry(req, NULL);

    return (ack == 0x01);
}


// Write 32-bit word aligned values to target memory using address auto-increment.
// size is in bytes.
static uint8_t jtag_write_block(uint32_t address, uint8_t *data, uint32_t size) {
    uint8_t tmp_in[4], req;
    uint32_t size_in_words;
    uint32_t i, ack;

    if (size==0)
        return 0;

    size_in_words = size/4;

    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }

    JTAG_IR(JTAG_APACC);

    // TAR write
    req = REG_AP | REG_W | (1 << 2);
    int2array(tmp_in, address, 4);
    if (jtag_transfer_retry(req, (uint32_t *)tmp_in) != 0x01) {
        return 0;
    }

    // DRW write
    req = REG_AP | REG_W | (3 << 2);
    for (i = 0; i < size_in_words; i++) {
        if (jtag_transfer_retry(req, (uint32_t *)data) != 0x01) {
            return 0;
        }
        data+=4;
    }

    // dummy read
    req = REG_DP | REG_R | REG_ADR(DP_RDBUFF);
    ack = jtag_transfer_retry(req, NULL);

    return (ack == 0x01);
}

// Read 32-bit word aligned values from target memory using address auto-increment.
// size is in bytes.
static uint8_t jtag_read_block(uint32_t address, uint8_t *data, uint32_t size) {
    uint8_t tmp_in[4], req, ack;
    uint32_t size_in_words;
    uint32_t i;

    if (size == 0) {
        return 0;
    }

    size_in_words = size/4;

    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }

    JTAG_IR(JTAG_APACC);

    // TAR write
    req = REG_AP | REG_W | (1 << 2);
    int2array(tmp_in, address, 4);
    if (jtag_transfer_retry(req, (uint32_t *)tmp_in) != 0x01) {
        return 0;
    }

    // read data
    req = REG_AP | REG_R | (3 << 2);
    // dummy read
    if (jtag_transfer_retry(req, (uint32_t *)data) != 0x01) {
        return 0;
    }

    for (i = 0; i< size_in_words; i++) {
        if (jtag_transfer_retry(req, (uint32_t *)data) != 0x01) {
            return 0;
        }
        data += 4;
    }

    // dummy read
    req = REG_DP | REG_R | REG_ADR(DP_RDBUFF);
    ack = jtag_transfer_retry(req, NULL);

    return (ack == 0x01);
}

// Read target memory.
static uint8_t jtag_read_data(uint32_t addr, uint32_t *val) {
    uint8_t tmp_in[4];
    uint8_t tmp_out[4];
    uint8_t req, ack;

    JTAG_IR(JTAG_APACC);

    // put addr in TAR register
    int2array(tmp_in, addr, 4);
    req = REG_AP | REG_W | (1 << 2);
    if (jtag_transfer_retry(req, (uint32_t *)tmp_in) != 0x01) {
        return 0;
    }

    // read data
    req = REG_AP | REG_R | (3 << 2);
    if (jtag_transfer_retry(req, (uint32_t *)tmp_out) != 0x01) {
        return 0;
    }

    // dummy read
    req = REG_DP | REG_R | REG_ADR(DP_RDBUFF);
    ack = jtag_transfer_retry(req, (uint32_t *)tmp_out);

    *val = (tmp_out[3] << 24) | (tmp_out[2] << 16) | (tmp_out[1] << 8) | tmp_out[0];

    return (ack == 0x01);
}

// Write target memory.
static uint8_t jtag_write_data(uint32_t address, uint32_t data) {
    uint8_t tmp_in[4];
    uint8_t req, ack;

    JTAG_IR(JTAG_APACC);

    // put addr in TAR register
    int2array(tmp_in, address, 4);
    req = REG_AP | REG_W | (1 << 2);
    if (jtag_transfer_retry(req, (uint32_t *)tmp_in) != 0x01) {
        return 0;
    }

    // write data
    int2array(tmp_in, data, 4);
    req = REG_AP | REG_W | (3 << 2);
    if (jtag_transfer_retry(req, (uint32_t *)tmp_in) != 0x01) {
        return 0;
    }

    // dummy read
    req = REG_DP | REG_R | REG_ADR(DP_RDBUFF);
    ack = jtag_transfer_retry(req, NULL);

    return (ack == 0x01) ? 1 : 0;
}

// Read 32-bit word from target memory.
static uint8_t jtag_read_word(uint32_t addr, uint32_t *val) {
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }

    if (!jtag_read_data(addr, val)) {
        return 0;
    }

    return 1;
}

// Write 32-bit word to target memory.
static uint8_t jtag_write_word(uint32_t addr, uint32_t val) {
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }

    if (!jtag_write_data(addr, val)) {
        return 0;
    }

    return 1;
}

// Read 8-bit byte from target memory.
static uint8_t jtag_read_byte(uint32_t addr, uint8_t *val) {
    uint32_t tmp;
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE8)) {
        return 0;
    }

    if (!jtag_read_data(addr, &tmp)) {
        return 0;
    }

    *val = (uint8_t)(tmp >> ((addr & 0x03) << 3));
    return 1;
}

// Write 8-bit byte to target memory.
static uint8_t jtag_write_byte(uint32_t addr, uint8_t val) {
    uint32_t tmp;

    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE8)) {
        return 0;
    }

    tmp = val << ((addr & 0x03) << 3);
    if (!jtag_write_data(addr, tmp)) {
        return 0;
    }

    return 1;
}

// Read unaligned data from target memory.
// size is in bytes.
uint8_t jtag_read_memory(uint32_t address, uint8_t *data, uint32_t size) {
    uint32_t n;

    // Read bytes until word aligned
    while ((size > 0) && (address & 0x3)) {
        if (!jtag_read_byte(address, data)) {
            return 0;
        }
        address++;
        data++;
        size--;
    }

    // Read word aligned blocks
    while (size > 3) {
        // Limit to auto increment page size
        n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (address & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));
        if (size < n) {
            n = size & 0xFFFFFFFC; // Only count complete words remaining
        }

        if (!jtag_read_block(address, data, n)) {
            return 0;
        }

        address += n;
        data += n;
        size -= n;
    }

    // Read remaining bytes
    while (size > 0) {
        if (!jtag_read_byte(address, data)) {
            return 0;
        }
        address++;
        data++;
        size--;
    }

    return 1;
}

// Write unaligned data to target memory.
// size is in bytes.
uint8_t jtag_write_memory(uint32_t address, uint8_t *data, uint32_t size) {
    uint32_t n;
    // Write bytes until word aligned
    while ((size > 0) && (address & 0x3)) {
        if (!jtag_write_byte(address, *data)) {
            return 0;
        }
        address++;
        data++;
        size--;
    }

    // Write word aligned blocks
    while (size > 3) {
        // Limit to auto increment page size
        n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (address & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));
        if (size < n) {
            n = size & 0xFFFFFFFC; // Only count complete words remaining
        }

        if (!jtag_write_block(address, data, n)) {
            return 0;
        }

        address += n;
        data += n;
        size -= n;
    }

    // Write remaining bytes
    while (size > 0) {
        if (!jtag_write_byte(address, *data)) {
            return 0;
        }
        address++;
        data++;
        size--;
    }

    return 1;
}

// Execute system call.
static uint8_t jtag_write_debug_state(DEBUG_STATE *state) {
    uint32_t i, status;

    if (!jtag_write_dp(DP_SELECT, 0)) {
        return 0;
    }

    // R0, R1, R2, R3
    for (i = 0; i < 4; i++) {
        if (!jtag_write_core_register(i, state->r[i])) {
            return 0;
        }
    }

    // R9
    if (!jtag_write_core_register(9, state->r[9])) {
        return 0;
    }

    // R13, R14, R15
    for (i=13; i<16; i++) {
        if (!jtag_write_core_register(i, state->r[i])) {
            return 0;
        }
    }

    // xPSR
    if (!jtag_write_core_register(16, state->xpsr)) {
        return 0;
    }

    if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
        return 0;
    }

    // check status
    if (!jtag_read_dp(DP_CTRL_STAT, &status)){
        return 0;
    }

    if (status & (STICKYERR | WDATAERR)) {
        return 0;
    }

    return 1;
}

static uint8_t jtag_read_core_register(uint32_t n, uint32_t *val) {
    int i = 0, timeout = 100;
    if (!jtag_write_word(DCRSR, n)) {
        return 0;
    }

    // wait for S_REGRDY
    for (i = 0; i < timeout; i++) {

        if (!jtag_read_word(DHCSR, val)) {
            return 0;
        }

        if (*val & S_REGRDY) {
            break;
        }
    }

    if (i == timeout) {
        return 0;
    }

    if (!jtag_read_word(DCRDR, val)) {
        return 0;
    }

    return 1;
}

static uint8_t jtag_write_core_register(uint32_t n, uint32_t val) {
    int i = 0, timeout = 100;
    if (!jtag_write_word(DCRDR, val))
        return 0;

    if (!jtag_write_word(DCRSR, n | REGWnR)) {
        return 0;
    }

    // wait for S_REGRDY
    for (i = 0; i < timeout; i++) {

        if (!jtag_read_word(DHCSR, &val)) {
            return 0;
        }

        if (val & S_REGRDY) {
            return 1;
        }
    }

    return 0;
}

uint8_t jtag_is_semihost_event(uint32_t *r0, uint32_t *r1) {
    uint32_t val;

    if (!jtag_read_word(DBG_HCSR, &val)) {
        return 0;
    }

    // Not hit breakpoint
    if ((val & S_HALT) == 0) {
        return 0;
    }

    // Has hit breakpoint
    // Read r0 and r1
    if (!jtag_read_core_register(0, r0)) {
        return 0;
    }

    if (!jtag_read_core_register(1, r1)) {
        return 0;
    }

    return 1;
}

static uint8_t jtag_wait_until_halted(void) {
    // Wait for target to stop
    uint32_t val, i, timeout = MAX_TIMEOUT;
    for (i = 0; i < timeout; i++) {

        if (!jtag_read_word(DBG_HCSR, &val)) {
            return 0;
        }

        if (val & S_HALT) {
            return 1;
        }
    }
    return 0;
}

// Restart target after BKPT
uint8_t jtag_semihost_restart(uint32_t r0) {
    uint32_t pc;

    // Update r0
    if (!jtag_write_core_register(0, r0)) {
        return 0;
    }

    // Update PC
    if (!jtag_read_core_register(15, &pc)) {
        return 0;
    }

    if (!jtag_write_core_register(15, pc + 2)) {
        return 0;
    }

    // Restart
    if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
        return 0;
    }

    return 1;
}

uint8_t jtag_flash_syscall_exec(const program_syscall_t *sysCallParam, uint32_t entry, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4) {
    DEBUG_STATE state;

    // Call flash algorithm function on target and wait for result.
    state.xpsr     = 0x01000000;          // xPSR: T = 1, ISR = 0
    state.r[0]     = arg1;                   // R0: Argument 1
    state.r[1]     = arg2;                   // R1: Argument 2
    state.r[2]     = arg3;                   // R2: Argument 3
    state.r[3]     = arg4;                   // R3: Argument 4

    state.r[9]     = sysCallParam->static_base;    // SB: Static Base

    state.r[13]    = sysCallParam->stack_pointer;  // SP: Stack Pointer
    state.r[14]    = sysCallParam->breakpoint;       // LR: Exit Point
    state.r[15]    = entry;                           // PC: Entry Point

    if (!jtag_write_debug_state(&state)) {
        return 0;
    }

    if (!jtag_wait_until_halted()) {
        return 0;
    }

    if (!jtag_read_core_register(0, &state.r[0])) {
        return 0;
    }

    // Flash functions return 0 if successful.
    if (state.r[0] != 0) {
        return 0;
    }

    return 1;
}


static uint8_t jtag_init_debug(void) {
    uint32_t tmp = 0;
    int retry;

    jtag_init();

    // init dap state with fake values
    dap_state.select = 0xffffffff;
    dap_state.csw = 0xffffffff;

    DAP_Setup();
    PORT_JTAG_SETUP();

    // call a target dependant function
    // this function can do several stuff before really
    // initing the debug
    target_before_init_debug();

    // Test-Logic-Reset
    PIN_SWDIO_TMS_SET();
    PIN_SWCLK_TCK_CLR();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_SET();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_CLR();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_SET();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_CLR();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_SET();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_CLR();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_SET();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_CLR();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_SET();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);

    // Run-Test-Idle
    PIN_SWDIO_TMS_CLR();
    PIN_SWCLK_TCK_CLR();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);
    PIN_SWCLK_TCK_SET();
    PIN_DELAY_SLOW(DAP_Data.clock_delay);

    // Read the IDCODE
    JTAG_IR(JTAG_IDCODE); // select JTAG chain
    tmp = JTAG_ReadIDCode();
    printf("IDCODE: %08x\n", tmp);

    // Ensure CTRL/STAT register selected in DPBANKSEL
    if (!jtag_write_dp(DP_SELECT, 0)) {
        return 0;
    }

    // Power up and clear errors
    if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | STICKYERR | STICKYCMP | STICKYORUN)) {
        return 0;
    }

    do {
        if (!jtag_read_dp(DP_CTRL_STAT, &tmp)) {
            return 0;
        }
    } while ((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) != (CDBGPWRUPACK | CSYSPWRUPACK));

    if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | TRNNORMAL | MASKLANE)) {
        return 0;
    }

    // call a target dependant function:
    // some target can enter in a lock state
    // this function can unlock these targets
    if (!target_unlock_sequence()) {
        return 0;
    }

    // Wait for debug access
    retry = 10;
    do {
        if (!jtag_read_ap(AP_CSW, &tmp)) {
            return 0;
        }

        if (retry-- == 0) {
            return 0;
        }
    } while (!(tmp & 0x40));

    if (!jtag_write_dp(DP_SELECT, 0)) {
        return 0;
    }

    return 1;
}


uint8_t jtag_set_target_state(TARGET_RESET_STATE state) {
    uint32_t val;
    int retry;

    switch (state) {
        case RESET_HOLD:
            PIN_nRESET_OUT(0);
            break;

        case RESET_RUN:
            PIN_nRESET_OUT(0);
            os_dly_wait(2);

            PIN_nRESET_OUT(1);
            os_dly_wait(2);
            break;

        case RESET_RUN_WITH_DEBUG:
            // First reset
            PIN_nRESET_OUT(0);
            os_dly_wait(1);

            PIN_nRESET_OUT(1);
            os_dly_wait(1);

            if (!jtag_init_debug()) {
                return 0;
            }

            // Enable debug (for semihost)
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
                return 0;
            }

            // Reset again
            PIN_nRESET_OUT(0);
            os_dly_wait(1);

            PIN_nRESET_OUT(1);
            os_dly_wait(1);
            break;

        case RESET_PROGRAM:
#if !defined(SOFT_RESET)
            // Use hardware reset (HW RESET)
            // First reset
            PIN_nRESET_OUT(0);
            os_dly_wait(2);

            PIN_nRESET_OUT(1);
            os_dly_wait(2);

            if (!jtag_init_debug()) {
//PIN_nRESET_OUT(1);
                return 0;
            }

            // Enable debug
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
//PIN_nRESET_OUT(1);
                return 0;
            }

            // Enable halt on reset
            if (!jtag_write_word(DBG_EMCR, VC_CORERESET)) {
//PIN_nRESET_OUT(1);
                return 0;
            }

            // Reset again
            PIN_nRESET_OUT(0);
            os_dly_wait(2);

            PIN_nRESET_OUT(1);
#else            
            if (!jtag_init_debug()) {
                return 0;
            }

            // Enable debug and halt the core (DHCSR <- 0xA05F0003)
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT)) {
                return 0;
            }
            
            // Wait until core is halted
            do {
                if (!jtag_read_word(DBG_HCSR, &val)) {
                    return 0;
                }
            } while((val & S_HALT) == 0);

            // Enable halt on reset
            if (!jtag_write_word(DBG_EMCR, VC_CORERESET)) {
                return 0;
            }

	        // Perform a soft reset
            if (!jtag_write_word(NVIC_AIRCR, VECTKEY | SOFT_RESET)) {
                return 0;
            }
#endif
            os_dly_wait(2);

            retry = 10;
            do {
                if (!jtag_read_word(DBG_HCSR, &val)) {
                    return 0;
                }

                if (retry-- == 0) {
                    if (!jtag_read_dp(DP_CTRL_STAT, &val)) {
                        return 0;
                    }
                    return 0;
                }
            } while((val & S_HALT) == 0);

            // Disable halt on reset
            if (!jtag_write_word(DBG_EMCR, 0)) {
                return 0;
            }

            break;

        case NO_DEBUG:
            if (!jtag_write_word(DBG_HCSR, DBGKEY)) {
                return 0;
            }
            break;

        case DEBUG:
            DAP_Setup();
            PORT_JTAG_SETUP();

            // Ensure CTRL/STAT register selected in DPBANKSEL
            if (!jtag_write_dp(DP_SELECT, 0)) {
                return 0;
            }

            // Power up
            if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) {
                return 0;
            }

            // Enable debug (for semihost)
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
                return 0;
            }

            break;

        default:
            return 0;
    }
    return 1;
}

#endif
