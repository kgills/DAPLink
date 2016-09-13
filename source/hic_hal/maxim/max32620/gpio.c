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

#include "max32620.h"
#include "clkman_regs.h"
#include "gpio_regs.h"
#include "RTL.h"
#include "DAP_config.h"

#define LED_PORT            4
#define DAP_LED_PIN         2
#define MSD_LED_PIN         0
#define CDC_LED_PIN         1

#define BUTTON_PORT         5
#define BUTTON_PIN          0

void gpio_init(void)
{
    uint32_t out_mode;

    /* Ensure that the GPIO clock is enabled */
    if (MXC_CLKMAN->sys_clk_ctrl_6_gpio == MXC_S_CLKMAN_CLK_SCALE_DISABLED) {
        MXC_CLKMAN->sys_clk_ctrl_6_gpio = MXC_S_CLKMAN_CLK_SCALE_DIV_1;
    }

    MXC_GPIO->out_val[LED_PORT] &= ~((0x1 << DAP_LED_PIN) |
        (0x1 << MSD_LED_PIN) |
        (0x1 << CDC_LED_PIN));

    // LED Outputs
    out_mode = MXC_GPIO->out_mode[LED_PORT];
    out_mode &= ~(0xF << (4 * DAP_LED_PIN));
    out_mode &= ~(0xF << (4 * MSD_LED_PIN));
    out_mode &= ~(0xF << (4 * CDC_LED_PIN));
    out_mode |= (MXC_V_GPIO_OUT_MODE_OPEN_DRAIN << (4 * DAP_LED_PIN));
    out_mode |= (MXC_V_GPIO_OUT_MODE_OPEN_DRAIN << (4 * MSD_LED_PIN));
    out_mode |= (MXC_V_GPIO_OUT_MODE_OPEN_DRAIN << (4 * CDC_LED_PIN));
    MXC_GPIO->out_mode[LED_PORT] = out_mode;

    // Button Inputs
    out_mode = MXC_GPIO->out_mode[BUTTON_PORT];
    out_mode &= ~(0xF << (4 * BUTTON_PIN));
    MXC_GPIO->out_mode[BUTTON_PORT] = out_mode;
    MXC_GPIO->in_mode[BUTTON_PORT] &= ~(0xF << (4 * BUTTON_PIN));
}

void gpio_set_hid_led(uint8_t state)
{
    if (state) {
        BITBAND_ClrBit(&MXC_GPIO->out_val[LED_PORT], DAP_LED_PIN);
    } else {
        BITBAND_SetBit(&MXC_GPIO->out_val[LED_PORT], DAP_LED_PIN);
    }
}

void gpio_set_msc_led(uint8_t state)
{
    if (state) {
        BITBAND_ClrBit(&MXC_GPIO->out_val[LED_PORT], MSD_LED_PIN);
    } else {
        BITBAND_SetBit(&MXC_GPIO->out_val[LED_PORT], MSD_LED_PIN);
    }
}

void gpio_set_cdc_led(uint8_t state)
{
    if (state) {
        BITBAND_ClrBit(&MXC_GPIO->out_val[LED_PORT], CDC_LED_PIN);
    } else {
        BITBAND_SetBit(&MXC_GPIO->out_val[LED_PORT], CDC_LED_PIN);
    }
}

uint8_t gpio_get_sw_reset(void)
{
    return BITBAND_GetBit(&MXC_GPIO->in_val[BUTTON_PORT], BUTTON_PIN);
}

