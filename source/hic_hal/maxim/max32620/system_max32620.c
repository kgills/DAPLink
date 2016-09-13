


/*******************************************************************************
* Copyright (C) 2014 Maxim Integrated Products, Inc., All Rights Reserved.
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
*******************************************************************************
*/

/* $Revision: 4184 $ $Date: 2015-01-30 11:52:14 -0600 (Fri, 30 Jan 2015) $ */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "max32620.h"
#include "clkman_regs.h"
#include "icc_regs.h"
#include "pwrseq_regs.h"
#include "pwrman_regs.h"
#include "adc_regs.h"
#include "rtc_regs.h"

// Set a default value for the Ring Oscillator Trim
#define UNTRIM

/* SCB CPACR Register Definitions */
/* Note: Added by Maxim Integrated, as these are missing from CMSIS/Core/Include/core_cm4.h */
#define SCB_CPACR_CP10_Pos                 20                   /*!< SCB CPACR: Coprocessor 10 Position */
#define SCB_CPACR_CP10_Msk                 (0x3UL << SCB_CPACR_CP10_Pos)                  /*!< SCB CPACR: Coprocessor 10 Mask */
#define SCB_CPACR_CP11_Pos                 22                   /*!< SCB CPACR: Coprocessor 11 Position */
#define SCB_CPACR_CP11_Msk                 (0x3UL << SCB_CPACR_CP11_Pos)                  /*!< SCB CPACR: Coprocessor 11 Mask */

#define __SYSTEM_CLOCK   RO_FREQ //--- Keil

/*----------------------------------------------------------------------------
  Base address of interrupt vector table in flash
 *----------------------------------------------------------------------------*/
extern void (* const isr_vector[])(void);

#if 0
/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __SYSTEM_CLOCK;  /*!< System Clock Frequency (Core Clock)*/
#endif

static uint8_t running;

// NOTE: Setting the cmsis SystemCoreClock value to the actual value it will
// be AFTER SystemInit() runs.  This is required so the hal drivers will have
// the correct value when the DATA sections are initalized.
uint32_t SystemCoreClock = RO_FREQ;

void DebugMon_Handler(void)
{
    __asm("BKPT 0");
}

void NMI_Handler(void)
{
#ifdef DEBUG_MODE
    printf("NMI\n");
#endif
    while(1)
        __WFI();
}

/*
 * TODO: f
 *
 * Expand debug printing information to include text of funct name
 * and location.  Print backtrace of stack.
 *
 */
#ifdef DEBUG_MODE
void FaultISR_C(uint32_t *hardfault_args)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;

    volatile unsigned char mmsr;
    volatile unsigned char bfsr;
    volatile unsigned short ufsr;
    volatile unsigned int hfsr;

    stacked_r0 = ((unsigned int) hardfault_args[0]);
    stacked_r1 = ((unsigned int) hardfault_args[1]);
    stacked_r2 = ((unsigned int) hardfault_args[2]);
    stacked_r3 = ((unsigned int) hardfault_args[3]);
    stacked_r12 = ((unsigned int) hardfault_args[4]);
    stacked_lr = ((unsigned int) hardfault_args[5]);
    stacked_pc = ((unsigned int) hardfault_args[6]);
    stacked_psr = ((unsigned int) hardfault_args[7]);

    printf("\n\n[Hard fault handler - all numbers in hex]\n");
    printf("R0 = 0x%08x\n", stacked_r0);
    printf("R1 = 0x%08x\n", stacked_r1);
    printf("R2 = 0x%08x\n", stacked_r2);
    printf("R3 = 0x%08x\n", stacked_r3);
    printf("R12 = 0x%08x\n", stacked_r12);
    printf("LR [R14] = 0x%08x  subroutine call return address\n", stacked_lr);
    printf("PC [R15] = 0x%08x  program counter address\n", stacked_pc);
    printf("PSR = 0x%08x\n", stacked_psr);
    printf("MMAR = 0x%08x  memory manage fault address\n", (*((volatile unsigned int *) (0xE000ED34))));
    printf("BFAR = 0x%08x  bus fault address\n", (*((volatile unsigned int *) (0xE000ED38))));

    /***********************************************************************************************
     * Memory Management Fault Status Register: (0xE000ED28)
     * Bit    Name         Description
     *  7     MMARVALID    MMAR is valid (0x40)
     *  4     MSTKERR      Stacking error (0x10)
     *  3     MUNSTKERR    Unstacking error (0x8)
     *  1     DACCVIOL     Data access violation (0x2)
     *  0     IACCVIOL     Instruction access violation (0x1)
     ***********************************************************************************************/
    mmsr = (*((volatile unsigned char *) (0xE000ED28)));
    printf("MMSR = 0x%02x  ", mmsr);
    if (mmsr & 0x40)
        printf("MMARVALID: MMAR is valid  ");
    if (mmsr & 0x10)
        printf("MSTKERR: Stacking error\n");
    else if (mmsr & 0x8)
        printf("MUNSTKERR: Unstacking error\n");
    else if (mmsr & 0x2)
        printf("DACCVIOL: Data access violation\n");
    else if (mmsr & 0x1)
        printf("IACCVIOL: Instruction access violation\n");
    else
        printf("\n");

    /***********************************************************************************************
     * Bus Fault Status Register: (0xE000ED28)
     * Bit    Name         Description                           
     *  7     BFARVALID    BFAR is valid (0x80)
     *  4     STKERR       Stacking error (0x10)
     *  3     UNSTKERR     Unstacking error (0x8)
     *  2     IMPREISERR  Imprecise data access violation (0x4)
     *  1     PRECISERR    Precise data access violation (0x2)
     *  0     IBUSERR      Instruction access violation (0x1)
     ***********************************************************************************************/
    bfsr = (*((volatile unsigned char *) (0xE000ED29)));
    printf("BFSR = 0x%02x  ", bfsr);
    if (bfsr & 0x80)
        printf("BFARVALID: BFAR is valid  ");
    if (bfsr & 0x10)
        printf("STKERR: Stacking error\n");
    else if (bfsr & 0x8)
        printf("UNSTKERR: Unstacking error\n");
    else if (bfsr & 0x4)
        printf("IMPREISERR: Imprecise data access violation\n");
    else if (bfsr & 0x2)
        printf("PRECISERR: Precise data access violation\n");
    else if (bfsr & 0x1)
        printf("IBUSERR: Instruction access violation\n");
    else
        printf("\n");

    /***********************************************************************************************
     * Usage Fault Status Register: (0xE000ED2A)
     * Bit    Name         Description
     *  9     DIVBYZERO    Divide by zero will take place (0x200)
     *  8     UNALIGNED    Unaligned access will take place (0x100)
     *  3     NOCP         Attempt to execute a coprocessor instruction (0x8)
     *  2     INVPC        Attempt to do exception with bad value (0x4)
     *  1     INVSTATE     Attempt to switch to invalid state (0x2)
     *  0     UNDEFINSTR   Attempt to execute an undefined instruction (0x1)
     ***********************************************************************************************/
    ufsr = (*((volatile unsigned short *) (0xE000ED2A)));
    printf("UFSR = 0x%04x  ", ufsr);
    if (ufsr & 0x200)
        printf("DIVBYZERO: Divide by zero will take place\n");
    else if (ufsr & 0x100)
        printf("UNALIGNED: Unaligned access will take place\n");
    else if (ufsr & 0x8)
        printf("NOCP: Attempt to execute a coprocessor instruction\n");
    else if (ufsr & 0x4)
        printf("INVPC: Attempt to do exception with bad value\n");
    else if (ufsr & 0x2)
        printf("INVSTATE: Attempt to switch to invalid state\n");
    else if (ufsr & 0x1)
        printf("UNDEFINSTR: Attempt to execute an undefined instruction\n");
    else
        printf("\n");

    /***********************************************************************************************
     * Usage Fault Status Register: (0xE000ED2A)
     * Bit    Name         Description
     * 31     DEBUGEVT     Hard fault caused by debug event (0x8000_0000)
     * 30     FORCED       Hard fault caused by bus/memory management/usage fault (0x4000_0000)
     *  1     VECTBL       Hard fault caused by failed vector fetch (0x1)
     ***********************************************************************************************/
    hfsr = (*((volatile unsigned int *) (0xE000ED2C)));
    printf("HFSR = 0x%08x  ", hfsr);
    if (hfsr & 0x80000000)
        printf("DEBUGEVT: Hard fault caused by debug event\n");
    else if (hfsr & 0x40000000)
        printf("FORCED: Hard fault caused by bus/memory management/usage fault\n");
    else if (hfsr & 0x1)
        printf("VECTBL: Hard fault caused by failed vector fetch\n");
    else
        printf("\n");

    //printf ("AFSR = 0x%08x\n", (*((volatile unsigned int *)(0xE000ED3C))));
    //printf ("SCB_SHCSR = %x\n", SCB->SHCSR);

    while (1) ;  /* Spin so we can use a debugger to anlayzer the situation */
}
#else /* DEBUG_MODE */
void FaultISR_C(uint32_t *hardfault_args)
{
    /* spin so we can use a debugger to anlayze the situation */
    while(1);

    /* reset the system */
    //NVIC_SystemReset();
}
#endif /* DEBUG_MODE */

void hard_fault_handler_c (unsigned int * hardfault_args)
{
  #ifdef DEBUG_MODE

  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;
 
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);
 
  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);
 
  printf ("\n\n[Hard fault handler - all numbers in hex]\n");
  printf ("R0 = %x\n", stacked_r0);
  printf ("R1 = %x\n", stacked_r1);
  printf ("R2 = %x\n", stacked_r2);
  printf ("R3 = %x\n", stacked_r3);
  printf ("R12 = %x\n", stacked_r12);
  printf ("LR [R14] = %x  subroutine call return address\n", stacked_lr);
  printf ("PC [R15] = %x  program counter\n", stacked_pc);
  printf ("PSR = %x\n", stacked_psr);
  printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
  printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
  printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
  printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
  printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
  printf ("SCB_SHCSR = %x\n", SCB->SHCSR);
  #endif 
  while (1);
}

void DefaultIRQ_Handler(void)
{
    #ifdef DEBUG_MODE
    printf("DF INT HANDLER\n");
    #endif

    /* spin so we can use a debugger to anlayze the situation */
    while(1);
}

void SystemCoreClockUpdate(void)
{
    switch ((MXC_CLKMAN->clk_ctrl & MXC_F_CLKMAN_CLK_CTRL_SYSTEM_SOURCE_SELECT) >> MXC_F_CLKMAN_CLK_CTRL_SYSTEM_SOURCE_SELECT_POS) {
             
        case MXC_V_CLKMAN_CLK_CTRL_SYSTEM_SOURCE_SELECT_96MHZ_RO_DIV_2:
        default:
            SystemCoreClock = RO_FREQ / 2;
            break;
        case MXC_V_CLKMAN_CLK_CTRL_SYSTEM_SOURCE_SELECT_96MHZ_RO:
            SystemCoreClock = RO_FREQ;
            break;
    }
}

#define MXC_F_RTC_OSC_CTRL_OSC_WARMUP_ENABLE (1 << 14)
void Trim_ROAtomic(void)
{
	uint32_t reg;
	uint32_t trim;

	// Step 1: enable 32KHz RTC
	running = MXC_PWRSEQ->reg0 & MXC_F_PWRSEQ_REG0_PWR_RTCEN_RUN;
	MXC_PWRSEQ->reg0 |= MXC_F_PWRSEQ_REG0_PWR_RTCEN_RUN;

	// Wait for RTC warm-up
	while (MXC_RTCCFG->osc_ctrl & MXC_F_RTC_OSC_CTRL_OSC_WARMUP_ENABLE);
	
	// Step 2: enable RO calibration complete interrupt
	MXC_ADC->intr |= MXC_F_ADC_INTR_RO_CAL_DONE_IE;

	// Step 3: clear RO calibration complete interuupt
	MXC_ADC->intr |= MXC_F_ADC_INTR_RO_CAL_DONE_IF;

	// Step 4: read RO flash trim shadow register*/
	// needed if parts are untrimmed
#ifdef UNTRIM
	reg = MXC_PWRSEQ->reg6;
	reg &= ~MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF;
	reg |= (480 << MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF_POS);
	MXC_PWRSEQ->reg6 = reg;
#endif

	// Step 5: write initial trim to frequency calibration initial condition register
	reg = MXC_ADC->ro_cal1;
	reg &= ~MXC_F_ADC_RO_CAL1_TRM_INIT;
	reg |= ((MXC_PWRSEQ->reg6 & MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF) >> MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF_POS) << MXC_F_ADC_RO_CAL1_TRM_INIT_POS;
	MXC_ADC->ro_cal1 = reg;

	// Step 6: load initial trim to active frequncy trim register
	MXC_ADC->ro_cal0 |= MXC_F_ADC_RO_CAL0_RO_CAL_LOAD;

	// Step 7: enable frequency loop to control RO trim
	MXC_ADC->ro_cal0 |= MXC_F_ADC_RO_CAL0_RO_CAL_EN;

	// Step 8: run frequency clibration in atomic mode
	MXC_ADC->ro_cal0 |= MXC_F_ADC_RO_CAL0_RO_CAL_ATOMIC;

	// Step 9: waiting for ro_cal_done flag
	while (!(MXC_ADC->intr & MXC_F_ADC_INTR_RO_CAL_DONE_IF));

	// Step 10: stop frequency calibration
	MXC_ADC->ro_cal0 &= ~MXC_F_ADC_RO_CAL0_RO_CAL_RUN;

	// Step 11: disable RO calibration complete interrupt
	MXC_ADC->intr &= ~MXC_F_ADC_INTR_RO_CAL_DONE_IE;

	// Step 12: read final frequency trim value
	trim = (MXC_ADC->ro_cal0 & MXC_F_ADC_RO_CAL0_RO_TRM) >> MXC_F_ADC_RO_CAL0_RO_TRM_POS;

	// Step 13: write final trim to RO flash trim shadow register
	reg = MXC_PWRSEQ->reg6;
	reg &= ~MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF;
	reg |= trim << MXC_F_PWRSEQ_REG6_PWR_TRIM_OSC_VREF_POS;
	MXC_PWRSEQ->reg6 = reg;

	// Step 14: restore RTC status
	if (running) {
		MXC_PWRSEQ->reg0 |= MXC_F_PWRSEQ_REG0_PWR_RTCEN_RUN;
	} else {
		MXC_PWRSEQ->reg0 &= ~MXC_F_PWRSEQ_REG0_PWR_RTCEN_RUN;
	}

	// Step 15: disable frequency loop to control RO trim
	MXC_ADC->ro_cal0 &= ~MXC_F_ADC_RO_CAL0_RO_CAL_EN;
}

void ICC_Enable(void)
{
	mxc_icc_regs_t *regs = MXC_ICC;

	// invalidate, wait, enable
	regs->invdt_all = 0xFFFF;
	while(!(regs->ctrl_stat & MXC_F_ICC_CTRL_STAT_READY));
	regs->ctrl_stat |= MXC_F_ICC_CTRL_STAT_ENABLE;

	// must invalidate a second time for proper use
	regs->invdt_all = 1;
}



/* This function can be implemented by the application to initialize the STDIO interface */
__attribute__((weak))
int stdio_init(void *base)
{
  // Do nothing
  return 0;
}

// This function to be implemented by the hal
extern void low_level_init(void);

void SystemInit(void)
{
	uint32_t reg;
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	/* Enable FPU on Cortex-M4, which occupies coprocessor slots 10 & 11 */
	/* Grant full access, per "Table B3-24 CPACR bit assignments". DDI0403D "ARMv7-M Architecture Reference Manual" */
	SCB->CPACR |= SCB_CPACR_CP10_Msk | SCB_CPACR_CP11_Msk;
	__DSB();
	__ISB();
#endif  

	// SCB->VTOR = (uint32_t)isr_vector;

	ICC_Enable();

	// Enable real-time clock during sleep mode
  MXC_PWRSEQ->reg0 |= (MXC_F_PWRSEQ_REG0_PWR_RTCEN_RUN | MXC_F_PWRSEQ_REG0_PWR_RTCEN_SLP);
	MXC_CLKMAN->clk_ctrl |= MXC_F_CLKMAN_CLK_CTRL_RTOS_MODE;
	// Turn on retention regulator - this needs some time to turn on before entering sleep
	MXC_PWRSEQ->reg0 |= (MXC_F_PWRSEQ_REG0_PWR_RETREGEN_RUN | MXC_F_PWRSEQ_REG0_PWR_RETREGEN_SLP);

	// Trim ring oscillator
	Trim_ROAtomic();

	// Select 96MHz ring oscillator as clock source
	reg = MXC_CLKMAN->clk_ctrl;
	reg &= ~MXC_F_CLKMAN_CLK_CTRL_SYSTEM_SOURCE_SELECT;
	reg |= 1 << MXC_F_CLKMAN_CLK_CTRL_SYSTEM_SOURCE_SELECT_POS;
	MXC_CLKMAN->clk_ctrl = reg;

	// The MAX32620 will not enter LP1 or LP0 if any of the GPIO WUD latches
	// are set, instead it will just wake up.  These are not cleared by reset,
	// so lets clear them here.
	// But only if a first boot event is detected, to avoid clearing valid
	// events when returning from LP0
	if ( (MXC_PWRSEQ->flags & MXC_F_PWRSEQ_FLAGS_PWR_FIRST_BOOT) ||
	    !(MXC_PWRMAN->pwr_rst_ctrl & MXC_F_PWRMAN_PWR_RST_CTRL_POR) ) {
	  MXC_PWRSEQ->reg1 |= (MXC_F_PWRSEQ_REG1_PWR_CLR_IO_EVENT_LATCH | MXC_F_PWRSEQ_REG1_PWR_CLR_IO_CFG_LATCH);
	  MXC_PWRSEQ->reg1 &= ~(MXC_F_PWRSEQ_REG1_PWR_CLR_IO_EVENT_LATCH | MXC_F_PWRSEQ_REG1_PWR_CLR_IO_CFG_LATCH);
	}

	// NOTE: These must be cleared before clearing IOWAKEUP
	MXC_PWRSEQ->reg1 |= MXC_F_PWRSEQ_REG1_PWR_CLR_IO_EVENT_LATCH;
	MXC_PWRSEQ->reg1 &= ~MXC_F_PWRSEQ_REG1_PWR_CLR_IO_EVENT_LATCH;

	MXC_PWRSEQ->flags |= MXC_F_PWRSEQ_FLAGS_PWR_IOWAKEUP;

	// Clear the firstboot bit, which is generated by a POR event and locks out LPx modes
	MXC_PWRSEQ->reg0 &= ~(MXC_F_PWRSEQ_REG0_PWR_FIRST_BOOT);

	// Clear all unused wakeup sources
	// Beware! Do not change any flag not mentioned here, as they will gate important power sequencer signals
	MXC_PWRSEQ->msk_flags &= ~(MXC_F_PWRSEQ_MSK_FLAGS_PWR_USB_PLUG_WAKEUP |
	                           MXC_F_PWRSEQ_MSK_FLAGS_PWR_USB_REMOVE_WAKEUP);

	// RTC sources are inverted, so a 1 will disable them
	MXC_PWRSEQ->msk_flags |= (MXC_F_PWRSEQ_MSK_FLAGS_RTC_CMPR1 |
	                          MXC_F_PWRSEQ_MSK_FLAGS_RTC_PRESCALE_CMP);

	// Unfreeze the GPIO by clearing MBUS_GATE
	// This is always safe to do, and restores our I/O when returning from LP0
	MXC_PWRSEQ->reg1 &= ~MXC_F_PWRSEQ_REG1_PWR_MBUS_GATE;

	stdio_init(NULL);
}
