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
#include "max32620.h"
#include "clkman_regs.h"
#include "gpio_regs.h"
#include "ioman_regs.h"
#include "pwrman_regs.h"
#include "DAP_config.h"

void stdio_init(void);

void board_init(void)
{
    uint32_t out_mode;

    /* Ensure that the GPIO clock is enabled */
    if (MXC_CLKMAN->sys_clk_ctrl_6_gpio == MXC_S_CLKMAN_CLK_SCALE_DISABLED) {
        MXC_CLKMAN->sys_clk_ctrl_6_gpio = MXC_S_CLKMAN_CLK_SCALE_DIV_1;
    }

    /* Assert C_RST_EN */
    MXC_CLRBIT(&MXC_GPIO->out_val[C_RST_EN_PORT], C_RST_EN_PIN);
    out_mode = MXC_GPIO->out_mode[C_RST_EN_PORT];
    out_mode &= ~(0xFU << (4 * C_RST_EN_PIN));
    out_mode |= (MXC_V_GPIO_OUT_MODE_NORMAL << (4 * C_RST_EN_PIN));
    MXC_GPIO->out_mode[C_RST_EN_PORT] = out_mode;

    MXC_SETBIT(&MXC_GPIO->out_val[EN_VDDIOH_PORT], EN_VDDIOH_PIN);
    out_mode = MXC_GPIO->out_mode[EN_VDDIOH_PORT];
    out_mode &= ~(0xFU << (4 * EN_VDDIOH_PIN));
    out_mode |= (MXC_V_GPIO_OUT_MODE_NORMAL << (4 * EN_VDDIOH_PIN));
    MXC_GPIO->out_mode[EN_VDDIOH_PORT] = out_mode;

    MXC_IOMAN->use_vddioh_0 = ((1 << ((TRGT_PORT * 8) + SRST_PIN)) |
                               (1 << ((TRGT_PORT * 8) + TCK_PIN)) |
                               (1 << ((TRGT_PORT * 8) + TMS_PIN)) |
                               (1 << ((TRGT_PORT * 8) + TDI_PIN)) |
                               (1 << ((SERIAL_PORT * 8) + SERIAL_TX_PIN)) |
                               (1 << ((SERIAL_PORT * 8) + SERIAL_RX_PIN)) |
                               (1 << ((TRGT_PORT * 8) + TDO_PIN)));
    MXC_IOMAN->use_vddioh_1 = (1 << (((OWM_PORT - 4) * 8) + OWM_PIN));

    MXC_GPIO->in_mode[OWM_PORT] &= ~(0xF << (4 * OWM_PIN));

    MXC_GPIO->in_mode[C_DETECT_PORT] &= ~(0xF << (4 * C_DETECT_PIN));

    MXC_GPIO->in_mode[TGT_DETECT_PORT] &= ~(0xF << (4 * TGT_DETECT_PIN));

    stdio_init();
}

int Board_Init(void)
{
    board_init();
    return 0;
}
