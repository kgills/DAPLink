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

#ifndef _OWM_MBED_H
#define _OWM_MBED_H

/* **** Includes **** */

#ifdef __cplusplus
extern "C" {
#endif

#define DS218EL_FAMILY          0x48
#define DS2431_FAMILY           0x2D
#define OW_ROMID_LEN            8

#define BOARD_ID_OFFSET         0
#define FLC_BASE_OFFSET         4
#define CLKDIV_OFFSET           8
#define FLASH_BASE_OFFSET       12
#define FLASH_SIZE_OFFSET       16
#define FLASH_SECTOR_OFFSET     20
#define SRAM_START_OFFSET       24
#define SRAM_END_OFFSET         28


typedef enum {
    OWM_MBED_LEVEL_NORMAL,
    OWM_MBED_LEVEL_STRONG
} owm_mbed_level_t;

typedef enum {
    OWM_MBED_SPEED_NORMAL,
    OWM_MBED_SPEED_OD
} owm_mbed_speed_t;

int owTouchReset(void);
unsigned owTouchBit(unsigned sendbit);
uint8_t owTouchByte(uint8_t sendbyte);
int owWriteByte(uint8_t sendbyte);
uint8_t owReadByte(void);
void owSpeed(owm_mbed_speed_t new_speed);
void owLevel(owm_mbed_level_t level);
void msDelay(int len);
long msGettick(void);
int owAcquire(void);
void owRelease(void);
int owReadRom(uint8_t *romid);

int owWriteMemDS28EL(uint8_t page, const uint8_t *data, unsigned len, uint8_t *romid);
int owReadMem32DS28EL(uint8_t page, uint8_t *data, uint8_t *romid);
int owWriteDS2431(uint16_t addr, const uint8_t* data, unsigned len, const uint8_t *romid);
int owReadDS2431(uint16_t addr, uint8_t* data, unsigned len, uint8_t *romid);

#ifdef __cplusplus
}
#endif

#endif /* _OWM_MBED_H */
