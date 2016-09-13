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

#include "FlashOS.h"        // FlashOS Structures
#include "max32620.h"
#include "flc_regs.h"


/******************************************************************************/
static int PrepareFLC(void)
{
  mxc_flc_regs_t *flc = MXC_FLC;

  /* Check if the flash controller is busy */
  if (flc->ctrl & MXC_F_FLC_CTRL_PENDING) {
    return 1;
  }

  /* Clear stale errors and disable interrupts */
  if (flc->intr != 0) {
    flc->intr = 0;
    if (flc->intr != 0) {
      flc->intr = MXC_F_FLC_INTR_FAILED_IF;
      if (flc->intr != 0) {
        return 1;
      }
    }
  }

  /* Unlock flash */
  flc->ctrl = (flc->ctrl & ~MXC_F_FLC_CTRL_FLSH_UNLOCK) | MXC_S_FLC_FLSH_UNLOCK_KEY;

  return 0;
}

/******************************************************************************/
uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
  mxc_flc_regs_t *flc = MXC_FLC;

  /* Check if the flash controller is busy */
  if (flc->ctrl & MXC_F_FLC_CTRL_PENDING) {
    return 1;
  }

  // Set flash clock divider to generate a 1MHz clock from the APB clock
  flc->fckdiv = SystemCoreClock/1000000;

  return  0;  // Finished without Errors
}

/******************************************************************************/
uint32_t UnInit(uint32_t fnc)
{
  mxc_flc_regs_t *flc = MXC_FLC;

  /* Lock flash */
  flc->ctrl &= ~MXC_F_FLC_CTRL_FLSH_UNLOCK;

  return  0;  // Finished without Errors
}

/******************************************************************************/
/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */
int EraseChip(void)
{
  mxc_flc_regs_t *flc = MXC_FLC;

  /* Prepare for the flash operation */
  if (PrepareFLC() != 0) {
    return 1; // Operation failed
  }

  /* Write mass erase code */
  flc->ctrl = (flc->ctrl & ~MXC_F_FLC_CTRL_ERASE_CODE) | MXC_S_FLC_ERASE_CODE_MASS_ERASE;

  /* Issue mass erase command */
  flc->ctrl |= MXC_F_FLC_CTRL_MASS_ERASE;

  /* Wait until flash operation is complete */
  while (flc->ctrl & MXC_F_FLC_CTRL_PENDING);

  /* Lock flash */
  flc->ctrl &= ~MXC_F_FLC_CTRL_FLSH_UNLOCK;

  /* Check access violations */
  if (flc->intr & MXC_F_FLC_INTR_FAILED_IF) {
    return 1; // Operation failed
  }

  return  0;  // Finished without Errors
}

/******************************************************************************/
/*
 *  Erase Sector in Flash Memory
 *    Parameter:      address:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
int EraseSector(unsigned long address)
{
  mxc_flc_regs_t *flc = MXC_FLC;

  /* Prepare for the flash operation */
  if (PrepareFLC() != 0) {
    return 1; // Operation failed
  }

  /* Write page erase code */
  flc->ctrl = (flc->ctrl & ~MXC_F_FLC_CTRL_ERASE_CODE) | MXC_S_FLC_ERASE_CODE_PAGE_ERASE;

  /* Issue page erase command */
  flc->faddr = address;
  flc->ctrl |= MXC_F_FLC_CTRL_PAGE_ERASE;

  /* Wait until flash operation is complete */
  while (flc->ctrl & MXC_F_FLC_CTRL_PENDING);

  /* Lock flash */
  flc->ctrl &= ~MXC_F_FLC_CTRL_FLSH_UNLOCK;

  /* Check access violations */
  if (flc->intr & MXC_F_FLC_INTR_FAILED_IF) {
    return 1; // Operation failed
  }

  return  0;  // Finished without Errors
}

/******************************************************************************/
/*
 *  Program Page in Flash Memory
 *    Parameter:      address:  Page Start Address
 *                    size:     Page Size
 *                    buffer:   Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
int ProgramPage(unsigned long address, unsigned long size, unsigned char *buffer8)
{
  mxc_flc_regs_t *flc = MXC_FLC;
  unsigned long remaining = size;
  unsigned long *buffer = (unsigned long *)buffer8;

  // Only accept 32-bit aligned pointers
  if ((unsigned long)buffer8 & 0x3) {
    return 1;
  }
  buffer = (unsigned long *)buffer8;

  /* Prepare for the flash operation */
  if (PrepareFLC() != 0) {
    return 1; // Operation failed
  }

  while (remaining >= 4) {
    flc->faddr = address;
    flc->fdata = *buffer++;
    flc->ctrl |= MXC_F_FLC_CTRL_WRITE_ENABLE;
    flc->ctrl |= MXC_F_FLC_CTRL_WRITE;

    /* Wait until flash operation is complete */
    while (flc->ctrl & MXC_F_FLC_CTRL_PENDING);

    address += 4;
    remaining -= 4;
  }

  if (remaining > 0) {
    uint32_t last_word;
    uint32_t mask;

    last_word = 0xffffffff;
    mask = 0xff;

    while (remaining > 0) {
      last_word &= (*buffer | ~mask);
      mask <<= 8;
      remaining--;
    }

    flc->faddr = address;
    flc->fdata = last_word;
    flc->ctrl |= MXC_F_FLC_CTRL_WRITE_ENABLE;

    /* Wait until flash operation is complete */
    while (flc->ctrl & MXC_F_FLC_CTRL_PENDING);
  }

  /* Lock flash */
  flc->ctrl &= ~MXC_F_FLC_CTRL_FLSH_UNLOCK;

  /* Check access violations */
  if (flc->intr & MXC_F_FLC_INTR_FAILED_IF) {
    return 1; // Operation failed
  }

  return  0;  // Finished without Errors
}
