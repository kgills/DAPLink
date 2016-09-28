/*******************************************************************************
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
 *******************************************************************************
 */
#include <string.h>
#include "max32625.h"
#include "clkman_regs.h"
#include "gpio_regs.h"
#include "ioman_regs.h"
#include "pwrman_regs.h"
#include "DAP_config.h"
#include "adc_regs.h"
#include "target_config.h"

void stdio_init(void);

void board_init(void)
{
    uint32_t out_mode;
    uint32_t int_mode;

    /* Ensure that the GPIO clock is enabled */
    if (MXC_CLKMAN->sys_clk_ctrl_6_gpio == MXC_S_CLKMAN_CLK_SCALE_DISABLED) {
        MXC_CLKMAN->sys_clk_ctrl_6_gpio = MXC_S_CLKMAN_CLK_SCALE_DIV_1;
    }

    // Enables the board to power VDDIOH and in tern the target micro

    // MXC_SETBIT(&MXC_GPIO->out_val[EN_VDDIOH_PORT], EN_VDDIOH_PIN);
    MXC_CLRBIT(&MXC_GPIO->out_val[EN_VDDIOH_PORT], EN_VDDIOH_PIN);
    out_mode = MXC_GPIO->out_mode[EN_VDDIOH_PORT];
    out_mode &= ~(0xFU << (4 * EN_VDDIOH_PIN));
    out_mode |= (MXC_V_GPIO_OUT_MODE_NORMAL << (4 * EN_VDDIOH_PIN));
    MXC_GPIO->out_mode[EN_VDDIOH_PORT] = out_mode;

    MXC_IOMAN->use_vddioh_0 = ((1 << ((TRGT_PORT * 8) + SRST_PIN)) |
                               (1 << ((TRGT_PORT * 8) + SWCLK_PIN)) |
                               (1 << ((TRGT_PORT * 8) + SWDIO_PIN)) |
                               (1 << ((TRGT_PORT * 8) + TX_PIN)) |
                               (1 << ((TRGT_PORT * 8) + RX_PIN)));

    // VDDIOH driver isn't strong enough with the strong pull-up
    MXC_IOMAN->use_vddioh_1 &= ~(1 << (((OWM_PORT - 4) * 8) + OWM_PIN));

    MXC_GPIO->in_mode[OWM_PORT] &= ~(0xF << (4 * OWM_PIN));

    // Button Inputs
    out_mode = MXC_GPIO->out_mode[BUTTON_PORT];
    out_mode &= ~(0xF << (4 * BUTTON_PIN));
    out_mode |= (0x2 << (4 * BUTTON_PIN));
    MXC_GPIO->out_mode[BUTTON_PORT] = out_mode;
    MXC_GPIO->out_val[BUTTON_PORT] |= (0x1 << BUTTON_PIN);
    MXC_GPIO->in_mode[BUTTON_PORT] &= ~(0xF << (4 * BUTTON_PIN));

    // Enable the interrupt
    int_mode = MXC_GPIO->int_mode[BUTTON_PORT];
    int_mode &= ~(0xF << (BUTTON_PIN*4));
    int_mode |= (MXC_V_GPIO_INT_MODE_FALLING_EDGE << (BUTTON_PIN*4));
    MXC_GPIO->int_mode[BUTTON_PORT] = int_mode;
    MXC_GPIO->intfl[BUTTON_PORT] = (0x1 << BUTTON_PIN);
    MXC_GPIO->inten[BUTTON_PORT] |= (0x1 << BUTTON_PIN);
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(BUTTON_PORT));

    // IOH_1W_EN
    MXC_SETBIT(&MXC_GPIO->out_val[2], 2);
    out_mode = MXC_GPIO->out_mode[2];
    out_mode &= ~(0xFU << (4 * 2));
    out_mode |= (MXC_V_GPIO_OUT_MODE_NORMAL << (4 * 2));
    MXC_GPIO->out_mode[2] = out_mode;

    // SWD_HDR_SEL
    MXC_SETBIT(&MXC_GPIO->out_val[2], 3);
    out_mode = MXC_GPIO->out_mode[2];
    out_mode &= ~(0xFU << (4 * 3));
    out_mode |= (MXC_V_GPIO_OUT_MODE_NORMAL << (4 * 3));
    MXC_GPIO->out_mode[2] = out_mode;

    // Strong pull-up disable
    MXC_SETBIT(&MXC_GPIO->out_val[4], 1);
    out_mode = MXC_GPIO->out_mode[4];
    out_mode &= ~(0xFU << (4 * 1));
    out_mode |= (MXC_V_GPIO_OUT_MODE_NORMAL << (4 * 1));
    MXC_GPIO->out_mode[4] = out_mode;

    // Setup the ADC

    /* Power up the ADC AFE, enable clocks */
    MXC_PWRMAN->pwr_rst_ctrl |= MXC_F_PWRMAN_PWR_RST_CTRL_AFE_POWERED;
    MXC_CLKMAN->clk_ctrl |= MXC_F_CLKMAN_CLK_CTRL_ADC_CLOCK_ENABLE;

    MXC_ADC->ctrl = (MXC_F_ADC_CTRL_ADC_PU |
        MXC_F_ADC_CTRL_ADC_CLK_EN |
        MXC_F_ADC_CTRL_BUF_PU |
        MXC_F_ADC_CTRL_ADC_REFBUF_PU |
        MXC_F_ADC_CTRL_ADC_CHGPUMP_PU);

    MXC_ADC->ctrl |= (MXC_F_ADC_CTRL_ADC_REFSEL);

    stdio_init();
}

int Board_Init(void)
{
    board_init();
    return 0;
}
