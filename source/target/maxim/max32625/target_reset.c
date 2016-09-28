/**
 * @file    target_reset.c
 * @brief   Target reset for the nrf51
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include "target_reset.h"
#include "swd_host.h"
#include "DAP_config.h"
#include "flash_intf.h"
#include "adc_regs.h"
#include "target_config.h"
#include "owm_mbed.h"

#define OW_DATA_LEN             32
#define DATA_PAGE               0

#define LOW_LIMIT               0x10
#define HIGH_LIMIT              0x10

extern char *board_id;
char new_board_id[5];

extern target_cfg_t target_device;
extern const uint32_t flash_algo_blob[];

static void delay_reset(void)
{
    volatile int i;
    for(i = 0; i < 0xFFFFFF; i++) {}
    // Reset the device
    NVIC_SystemReset();

    while (1); // Wait for reset
}

void read_device(int boot)
{
    char board_id_temp[5];
    uint8_t romid[OW_ROMID_LEN];
    uint8_t buffer[OW_DATA_LEN];
    uint32_t flash_start, flash_size, flash_sector;

    // Initialize the OW device
    if(owAcquire() != 0) {
        // Error acquiring the ow device
        if(!boot) {
            delay_reset();
        }
        return;
    }

    // Read the ROMID
    owSpeed(OWM_MBED_SPEED_NORMAL);
    if(owReadRom(romid) != 0) {
        // Try an OD read ROMID
        owSpeed(OWM_MBED_SPEED_OD);
        if(owReadRom(romid) != 0) {
            // Can't get ROMID
            if(!boot) {
                // Reset to load the default settings
                delay_reset();
            }

            return; 
        }
    }

    // Read the data from the ow device
    if(romid[0] == DS2431_FAMILY) {
        if(owReadDS2431(DATA_PAGE, buffer, OW_DATA_LEN, romid) != 0) {
            return;
        }
    } else if(romid[0] == DS218EL_FAMILY) {
        // Reads 32 bytes from the given page
        if(owReadMem32DS28EL(DATA_PAGE, buffer, romid) != 0) {
            return;
        }  
    } else {
        // Unknown OW device attached
        return;
    }

    memcpy(board_id_temp, &buffer[BOARD_ID_OFFSET], 4);
    board_id_temp[4] = '\0';

    // Currently board matches what we read
    if(strcmp(board_id, board_id_temp) == 0) {
        return;
    }

    memcpy(&flash_size, &buffer[FLASH_SIZE_OFFSET], 4);
    if(flash_size == 0x0 || flash_size == 0xFFFFFFFF) {
        // OW device not programmed
        if(!boot) {
            // Reset to load the default settings
            delay_reset();
        }
        return; 
    }


    // Reset the device if this was triggered by the PB
    if(!boot) {
        delay_reset();        
    }

    memcpy(new_board_id, board_id_temp, 4);
    board_id = new_board_id;

    memcpy(&target_device.ram_start, &buffer[SRAM_START_OFFSET], 4);
    memcpy(&target_device.ram_end, &buffer[SRAM_END_OFFSET], 4);
    memcpy(&flash_start, &buffer[FLASH_BASE_OFFSET], 4);
    memcpy(&flash_sector, &buffer[FLASH_SECTOR_OFFSET], 4);

    target_device.sector_cnt = flash_size / flash_sector;
    target_device.flash_start = flash_start;
    target_device.flash_end = flash_start + flash_size;

    memcpy((void*)(flash_algo_blob+163), &flash_start, 4);
    memcpy((void*)(flash_algo_blob+164), &flash_size, 4);
    memcpy((void*)(flash_algo_blob+165), &flash_sector, 4);
}

void prerun_target_config(void)
{
    uint32_t ctrl_tmp;
    volatile uint16_t reading;

    // Read the device that's currently connected
    read_device(1);

    // Initialize the analog input to sense the plug/unplug

    /* Insert channel selection */
    ctrl_tmp = MXC_ADC->ctrl;
    ctrl_tmp &= ~(MXC_F_ADC_CTRL_ADC_CHSEL);
    ctrl_tmp |= ((MXC_V_ADC_CTRL_ADC_CHSEL_VDDIOH_DIV_4 << MXC_F_ADC_CTRL_ADC_CHSEL_POS) & MXC_F_ADC_CTRL_ADC_CHSEL);

    /* Clear channel configuration */
    ctrl_tmp &= ~(MXC_F_ADC_CTRL_ADC_REFSCL | MXC_F_ADC_CTRL_ADC_SCALE | MXC_F_ADC_CTRL_BUF_BYPASS);

    /* ADC reference scaling must be set for all channels but two*/
    ctrl_tmp |= MXC_F_ADC_CTRL_ADC_REFSCL;
    ctrl_tmp |= MXC_F_ADC_CTRL_ADC_SCALE;
    ctrl_tmp |= MXC_F_ADC_CTRL_BUF_BYPASS;

    /* Write this configuration */
    MXC_ADC->ctrl = ctrl_tmp;

#if 1
    // Test the readings
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_CPU_ADC_START;

    while ((MXC_ADC->intr & MXC_F_ADC_INTR_ADC_DONE_IF) == 0);
    reading = (uint16_t)(MXC_ADC->data);
#endif

    // Setup the limits
    MXC_ADC->limit[0] = ((MXC_V_ADC_CTRL_ADC_CHSEL_VDDIOH_DIV_4 << MXC_F_ADC_LIMIT0_CH_SEL_POS) & MXC_F_ADC_LIMIT0_CH_SEL);

    MXC_ADC->limit[0] |= MXC_F_ADC_LIMIT0_CH_LO_LIMIT_EN |
        ((LOW_LIMIT << MXC_F_ADC_LIMIT0_CH_LO_LIMIT_POS) & MXC_F_ADC_LIMIT0_CH_LO_LIMIT);

    MXC_ADC->limit[0] |= MXC_F_ADC_LIMIT0_CH_HI_LIMIT_EN |
        ((HIGH_LIMIT << MXC_F_ADC_LIMIT0_CH_HI_LIMIT_POS) & MXC_F_ADC_LIMIT0_CH_HI_LIMIT);

}

void ADC_IRQHandler(void)
{

}

void GPIO_P2_IRQHandler(void)
{
    uint8_t intfl;

    // Read and clear enabled interrupts.
    intfl = MXC_GPIO->intfl[2];
    intfl &= MXC_GPIO->inten[2];
    MXC_GPIO->intfl[2] = intfl;

    if(intfl & (0x1 << BUTTON_PIN)) {
        read_device(0);
    }
}

void target_before_init_debug(void)
{
    // any target specific sequences needed before attaching
    // to the DAP across JTAG or SWD
    return;
}

uint8_t target_unlock_sequence(void)
{
    // if the device can secure the flash and there is a way to
    //  erase all it should be implemented here.
    return 1;
}

uint8_t target_set_state(TARGET_RESET_STATE state)
{
    // invoke reset by sw (VECT_REQ or SYS_REQ) or hw (hardware IO toggle)
    //return swd_set_target_state_sw(state);
    //or 


    return swd_set_target_state_hw(state);
}

uint8_t security_bits_set(uint32_t addr, uint8_t *data, uint32_t size)
{
    // if there are security bits in the programmable flash region
    //  a check should be performed. This method is used when programming
    //  by drag-n-drop and should refuse to program an image requesting
    //  to set the device security. This can be performed with the debug channel
    //  if needed.
    return 0;
}
