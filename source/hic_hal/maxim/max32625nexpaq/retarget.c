/******************************************************************************/
/* RETARGET.C: 'Retarget' layer for target-dependent low level functions      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005 Keil Software. All rights reserved.                     */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <stdio.h>
#include <rt_misc.h>

#include "max32620.h"
#include "clkman_regs.h"
#include "ioman_regs.h"
#include "uart_regs.h"
#include "gpio_regs.h"

// Allow baud rate to be overridden at build time
#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD  9600
#endif

#pragma import(__use_no_semihosting_swi)

struct __FILE {
    int handle; /* Add whatever you need here */
};
FILE __stdout;
FILE __stdin;

/******************************************************************************/
static void set_baudrate(void)
{
    uint32_t baud_setting;

    baud_setting = SystemCoreClock / (1 << (MXC_CLKMAN->sys_clk_ctrl_8_uart - 1));
    baud_setting /= (CONSOLE_BAUD * 16);
    MXC_UART3->baud = baud_setting;
}

/******************************************************************************/
void stdio_init(void)
{
    // Enable clock and select prescaler
    MXC_CLKMAN->clk_gate_ctrl1 |= (2 << MXC_F_CLKMAN_CLK_GATE_CTRL1_UART3_CLK_GATER_POS);
    if (MXC_CLKMAN->sys_clk_ctrl_8_uart != MXC_S_CLKMAN_CLK_SCALE_DIV_4) {
        MXC_CLKMAN->sys_clk_ctrl_8_uart = MXC_S_CLKMAN_CLK_SCALE_DIV_4;
    }

    // Configure GPIO for UART
    MXC_IOMAN->uart3_req = ((MXC_V_IOMAN_MAP_B << MXC_F_IOMAN_UART3_REQ_IO_MAP_POS) | MXC_F_IOMAN_UART3_REQ_IO_REQ);
    while (MXC_IOMAN->uart3_ack != ((MXC_V_IOMAN_MAP_B << MXC_F_IOMAN_UART3_REQ_IO_MAP_POS) | MXC_F_IOMAN_UART3_REQ_IO_REQ));

    // Flush RX and TX FIFOS
    MXC_UART3->ctrl &= ~(MXC_F_UART_CTRL_RX_FIFO_EN | MXC_F_UART_CTRL_TX_FIFO_EN);
    MXC_UART3->ctrl |= (MXC_F_UART_CTRL_RX_FIFO_EN | MXC_F_UART_CTRL_TX_FIFO_EN);

    // Disable interrupts
    MXC_UART3->inten = 0;
    MXC_UART3->intfl = MXC_UART3->intfl;

    // Set the parity, size, stop and flow configuration
    MXC_UART3->ctrl |= (MXC_S_UART_CTRL_DATA_SIZE_8_BITS | MXC_S_UART_CTRL_PARITY_DISABLE);

    // Set the baud rate
    set_baudrate();

    MXC_UART3->ctrl |= MXC_F_UART_CTRL_UART_EN;
}

/******************************************************************************/
int fputc(int ch, FILE *f)
{
    if (ch == '\n') {
        // Wait for TXFIFO to not be full
        while ((MXC_UART3->tx_fifo_ctrl & MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY) == MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY);
        MXC_UART3_FIFO->tx = '\r';
    }

    // Wait for TXFIFO to not be full
    while ((MXC_UART3->tx_fifo_ctrl & MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY) == MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY);
    MXC_UART3_FIFO->tx = ch;

    return ch;
}

/******************************************************************************/
int fgetc(FILE *f)
{
    char ch;

    // Wait for data to be available
    while (!(MXC_UART3->rx_fifo_ctrl & MXC_F_UART_RX_FIFO_CTRL_FIFO_ENTRY));

    ch = MXC_UART3_FIFO->rx & (uint8_t)0xFF;
    fputc(ch, f);

    return ch;
}

/******************************************************************************/
int ferror(FILE *f)
{
    /* Your implementation of ferror */
    return EOF;
}

/******************************************************************************/
void _ttywrch(int ch)
{
    // Wait for TXFIFO to not be full
    while ((MXC_UART3->tx_fifo_ctrl & MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY) == MXC_F_UART_TX_FIFO_CTRL_FIFO_ENTRY);
    MXC_UART3_FIFO->tx = ch;
}

/******************************************************************************/
void _sys_exit(int return_code)
{
    while (1);    /* endless loop */
}
