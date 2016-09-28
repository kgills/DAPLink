/* ****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 **************************************************************************** */

#include <string.h>
#include "max32625.h"
#include "owm_regs.h"
#include "owm_mbed.h"
#include "DAP.h"
#include "clkman_regs.h"
#include "ioman_regs.h"
#include "gpio_regs.h"
#include "IO_Config.h"

#define READ_ROM_CMD            0x33
#define WRITE_MEMORY_CMD        0x55
#define READ_MEMORY_CMD         0xF0
#define MATCH_ROM_CMD           0x55
#define SKIP_ROM_CMD            0x3C
#define RELEASE_CMD             0xAA
#define READ_SCRATCH_CMD        0xAA
#define WRITE_SCRATCH_CMD       0x0F
#define COPY_SCRATCH_CMD        0x55
#define COPY_SCRATCH_RET        0xAA

#define TPRD_MS                 10
#define WRITE_RET               0xAA
#define CRC16_RET               0xB001

// *****************************************************************************
int owTouchReset(void)
{
    MXC_OWM->ctrl_stat |= MXC_F_OWM_CTRL_STAT_START_OW_RESET;       // Generate a reset pulse and look for reply
    while((MXC_OWM->intfl & MXC_F_OWM_INTFL_OW_RESET_DONE) == 0);   // Wait for reset time slot to complete
    MXC_OWM->intfl = MXC_F_OWM_INTFL_OW_RESET_DONE;                 // Clear the flag
    return (MXC_OWM->ctrl_stat & MXC_F_OWM_CTRL_STAT_PRESENCE_DETECT) ? 0 : -1;
}

// *****************************************************************************
unsigned owTouchBit(unsigned sendbit)
{
    MXC_OWM->cfg |= MXC_F_OWM_CFG_SINGLE_BIT_MODE;                  // Send a single bit
    MXC_OWM->data = (sendbit << MXC_F_OWM_DATA_TX_RX_POS) & MXC_F_OWM_DATA_TX_RX;       // Write data
    while((MXC_OWM->intfl & MXC_F_OWM_INTFL_TX_DATA_EMPTY) == 0);   // Wait for data to be sent
    while((MXC_OWM->intfl & MXC_F_OWM_INTFL_RX_DATA_READY) == 0);   // Wait for data to be read
    MXC_OWM->intfl = (MXC_F_OWM_INTFL_TX_DATA_EMPTY | MXC_F_OWM_INTFL_RX_DATA_READY);   // Clear the flags
    return (MXC_OWM->data >> MXC_F_OWM_DATA_TX_RX_POS) & 1;
}

// *****************************************************************************
uint8_t owTouchByte(uint8_t sendbyte)
{
    MXC_OWM->cfg &= ~MXC_F_OWM_CFG_SINGLE_BIT_MODE;                 // Send 8 bits
    MXC_OWM->data = (sendbyte << MXC_F_OWM_DATA_TX_RX_POS) & MXC_F_OWM_DATA_TX_RX;      // Write data
    while((MXC_OWM->intfl & MXC_F_OWM_INTFL_TX_DATA_EMPTY) == 0);   // Wait for data to be sent
    while((MXC_OWM->intfl & MXC_F_OWM_INTFL_RX_DATA_READY) == 0);   // Wait for data to be read
    MXC_OWM->intfl = (MXC_F_OWM_INTFL_TX_DATA_EMPTY | MXC_F_OWM_INTFL_RX_DATA_READY);   // Clear the flags
    return (MXC_OWM->data >> MXC_F_OWM_DATA_TX_RX_POS) & 0xFF;
}

// *****************************************************************************
int owWriteByte(uint8_t sendbyte)
{
   return (owTouchByte(sendbyte) == sendbyte) ? 0 : -1;
}

// *****************************************************************************
uint8_t owReadByte(void)
{
   return owTouchByte(0xFF);
}

// *****************************************************************************
void owReadBlock(uint8_t *buffer, unsigned len)
{   
    int i;
    for(i = 0; i < len; i++) {
        buffer[i] = owReadByte();
    }
}

// *****************************************************************************
int owWriteBlock(const uint8_t *buffer, unsigned len)
{   
    int i;
    for(i = 0; i < len; i++) {
        if(owWriteByte(buffer[i]) != 0) {
            return -1;
        }
    }

    return 0;
}

// *****************************************************************************
void owSpeed(owm_mbed_speed_t new_speed)
{
    if(new_speed == OWM_MBED_SPEED_NORMAL) {
        MXC_OWM->cfg &= ~MXC_F_OWM_CFG_OVERDRIVE;   // Set standard speed
    } else {
        MXC_OWM->cfg |= MXC_F_OWM_CFG_OVERDRIVE;    // Set overdrive speed
    }
}

// *****************************************************************************
void owLevel(owm_mbed_level_t level)
{
   // this driver only supports normal 1-wire levels.
}

// *****************************************************************************
void msDelay(int len)
{
    Delayms(len);
}

// *****************************************************************************
int owAcquire(void)
{
    uint32_t freq;

    uint32_t out_mode;
    
    //only request for the IO pin
    MXC_IOMAN->owm_req = MXC_F_IOMAN_OWM_REQ_MAPPING_REQ;
    while(MXC_IOMAN->owm_ack != MXC_F_IOMAN_OWM_ACK_MAPPING_ACK) {}

    // Turn on the clock
    MXC_CLKMAN->sys_clk_ctrl_15_owm = MXC_V_CLKMAN_CLK_SCALE_DIV_1;
    freq = SystemCoreClock;

    // Set divisor to generate 1MHz clock
    freq /= 1000000;
    MXC_OWM->clk_div_1us = (freq << MXC_F_OWM_CLK_DIV_1US_DIVISOR_POS) & MXC_F_OWM_CLK_DIV_1US_DIVISOR;

    //enabled internal pullup on IO pin
    // MXC_OWM->cfg |= MXC_F_OWM_CFG_INT_PULLUP_ENABLE;


    // Clear all interrupt flags
    MXC_OWM->intfl = (MXC_F_OWM_INTFL_OW_RESET_DONE |
                      MXC_F_OWM_INTFL_TX_DATA_EMPTY |
                      MXC_F_OWM_INTFL_RX_DATA_READY);

    return 0;
}

// *****************************************************************************
uint8_t calculateCrcByte(uint8_t crc8, uint8_t data)
{
    int i;
    // See Application Note 27
    crc8 = crc8 ^ data;
    for (i = 0; i < 8; i++)
    {
        if (crc8 & 1)
        {
            crc8 = (crc8 >> 1) ^ 0x8c;
        }
        else
        {
            crc8 = (crc8 >> 1);
        }
    }
 
    return crc8;
}


// *****************************************************************************
uint8_t calculateCrc8(const uint8_t * data, unsigned dataLen, uint8_t crc)
{
    int i;
    for (i = 0; i < dataLen; i++)
    {
        crc = calculateCrcByte(crc, data[i]);
    }
    return crc;
}

// *****************************************************************************       
uint16_t calculateCrc16Byte(uint16_t crc16, uint16_t data)
{
    const uint16_t oddparity[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    data = (data ^ (crc16 & 0xff)) & 0xff;
    crc16 >>= 8;

    if (oddparity[data & 0xf] ^ oddparity[data >> 4])
    {
        crc16 ^= 0xc001;
    }

    data <<= 6;
    crc16 ^= data;
    data <<= 1;
    crc16 ^= data;

    return crc16;
}

// *****************************************************************************
uint16_t calculateCrc16(const uint8_t * data, unsigned dataOffset, unsigned dataLen, uint16_t crc)
{
    int i;
    for (i = dataOffset; i < (dataLen + dataOffset); i++)
    {
        crc = calculateCrc16Byte(crc, data[i]);
    }
    return crc;
}

// *****************************************************************************
int owReadRom(uint8_t *romid)
{
    // Reset and detect if there is a device present
    if(owTouchReset() != 0) {
        return -1;
    }

    // Write the read ROM command
    if(owWriteByte(READ_ROM_CMD) != 0) {
        return -1;
    }

    // Read block to read the ROM
    owReadBlock(romid, OW_ROMID_LEN);

    // Verify the CRC
    if(calculateCrc8(romid, 7, 0x00) != romid[7]) {
        return -1;
    }


    return 0;
}

// *****************************************************************************
int owWriteMemDS28EL(uint8_t page, const uint8_t *data, unsigned len, uint8_t *romid)
{
    uint8_t buffer[6];
    int i;

    buffer[0] = WRITE_MEMORY_CMD;
    buffer[1] = page;

    if(len > 32) {
        return -1;
    }

    if(len%4 != 0) {
        return -1;
    }

    // Reset and detect if there is a device present
    if(owTouchReset() != 0) {
        return -1;
    }

    owWriteByte(MATCH_ROM_CMD);
    owWriteBlock(romid, OW_ROMID_LEN);

    if(owWriteBlock(buffer, 2) != 0) {
        return -1;
    }

    // Verify CRC    
    owReadBlock(&buffer[2], 2);
    if(calculateCrc16(buffer, 0, 4, 0x0000) != CRC16_RET) {
        return -1;
    }

    for(i = 0; i < len; i+=4) {

        // Write 4 bytes at time
        owWriteBlock(&data[i], 4);

        owReadBlock(&buffer[4], 2);

        // Check the CRC
        memcpy(buffer, &data[i], 4);
        if(calculateCrc16(buffer, 0, 6, 0x0000) != CRC16_RET) {
            return -1;
        }

        // Send the release command
        owWriteByte(RELEASE_CMD);

        // Wait for the device to write the memory
        msDelay(TPRD_MS);

        // Check the return value
        buffer[0] = owReadByte();
        if(buffer[0] != WRITE_RET) {
            return -1;
        }
    }


    return 0;
}

// *****************************************************************************
int owReadMem32DS28EL(uint8_t page, uint8_t *data, uint8_t *romid)
{
    uint8_t buffer[4];

    buffer[0] = READ_MEMORY_CMD;
    buffer[1] = page;

    // Reset and detect if there is a device present
    if(owTouchReset() != 0) {
        return -1;
    }

    owWriteByte(MATCH_ROM_CMD);
    owWriteBlock(romid, OW_ROMID_LEN);

    if(owWriteBlock(buffer, 2) != 0) {
        return -1;
    }
    
    owReadBlock(&buffer[2], 2);

    // Verify CRC
    if(calculateCrc16(buffer, 0, 4, 0x0000) != CRC16_RET) {
        return -1;
    }

    owReadBlock(data, 32);

    return 0;
}

// Read 8 bytes from the scratchpad
int owReadScratch(uint16_t addr, uint8_t *data, uint8_t *es, const uint8_t *romid)
{
    uint8_t buffer[13];
    uint16_t inv_crc, inv_crc_calc;
    unsigned index;

    buffer[0] = READ_SCRATCH_CMD;

    // Reset and detect if there is a device present
    if(owTouchReset() != 0) {
        return -1;
    }

    owWriteByte(MATCH_ROM_CMD);
    owWriteBlock(romid, OW_ROMID_LEN);

    if(owWriteBlock(buffer, 1) != 0) {
        return -1;
    }
    
    owReadBlock(&buffer[0], 13);

    inv_crc = ((buffer[12] << 8) | buffer[11]);
            
    index = 12;
    while(--index)
    {
        buffer[index] = buffer[index - 1];
    }
    buffer[0] = READ_SCRATCH_CMD;
    
    //calc our own inverted CRC16 to compare with one returned
    inv_crc_calc = ~calculateCrc16(buffer, 0, 12, 0x0000);
    
    if(inv_crc == inv_crc_calc)
    {
        *es = buffer[3];
        memcpy(data, (buffer + 4), 8);

        return 0;
    }

    return -1;
}

// Write 8 bytes to the sratchpad
int owWriteScratch(uint16_t addr, const uint8_t *data, const uint8_t *romid)
{
    uint8_t buffer[11];
    uint16_t inv_crc, inv_crc_calc;

    // Reset and detect if there is a device present
    if(owTouchReset() != 0) {
        return -1;
    }

    owWriteByte(MATCH_ROM_CMD);
    owWriteBlock(romid, OW_ROMID_LEN);


    buffer[0] = WRITE_SCRATCH_CMD;
    buffer[1] = addr & 0xFF;
    buffer[2] = (addr >> 8) & 0xFF;
    memcpy(&buffer[3], data, 8);

    if(owWriteBlock(buffer, 11) != 0) {
        return -1;
    }

    owReadBlock((uint8_t*)&inv_crc, 2);
            
    //calc our own inverted CRC16 to compare with one returned
    inv_crc_calc = ~calculateCrc16(buffer, 0, 11, 0x0000);
    
    if(inv_crc == inv_crc_calc) {
        return 0;
    }

    return -1;
}

int owCopyScratch(uint16_t addr, uint8_t es, const uint8_t *romid)
{
    uint8_t buffer[4];

    // Reset and detect if there is a device present
    if(owTouchReset() != 0) {
        return -1;
    }

    owWriteByte(MATCH_ROM_CMD);
    owWriteBlock(romid, OW_ROMID_LEN);

    buffer[0] = COPY_SCRATCH_CMD;
    buffer[1] = addr & 0xFF;
    buffer[2] = (addr >> 8) & 0xFF;
    buffer[3] = es;

    if(owWriteBlock(buffer, 4) != 0) {
        return -1;
    }

    msDelay(TPRD_MS);

    owReadBlock(buffer, 1);

    if(buffer[0] != COPY_SCRATCH_RET) {
        return -1;
    }

    return 0;
}


int owWriteDS2431(uint16_t addr, const uint8_t* data, unsigned len, const uint8_t *romid)
{
    uint8_t es;
    uint8_t buffer[8];
    uint8_t i;

    if((len % 8) != 0) {
        return -1;
    }

    // For on length
    for(i = addr; i < (addr+len); i+=8) {

        // Write scratch
        if(owWriteScratch(i, &data[i - addr], romid) != 0) {
            return -1;
        }

        // Read scratch
        if(owReadScratch(i, buffer, &es, romid) != 0) {
            return -1;
        }

        // Copy scratch
        if(owCopyScratch(i, es, romid) != 0) {
            return -1;
        }
    }   

    return 0;
}

int owReadDS2431(uint16_t addr, uint8_t* data, unsigned len, uint8_t *romid)
{
    uint8_t buffer[3];

    buffer[0] = READ_MEMORY_CMD;
    buffer[1] = addr & 0xFF;
    buffer[2] = (addr >> 8) & 0xFF;

    // Reset and detect if there is a device present
    if(owTouchReset() != 0) {
        return -1;
    }

    owWriteByte(MATCH_ROM_CMD);
    owWriteBlock(romid, OW_ROMID_LEN);

    if(owWriteBlock(buffer, 3) != 0) {
        return -1;
    }
    
    owReadBlock(&data[0], len);

    return 0;
}
