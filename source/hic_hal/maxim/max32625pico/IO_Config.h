/**
 * @file    IO_Config.h
 * @brief
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

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "max32625.h"

#define CDC_ACM_UART        2

#define TRGT_PORT           3
#define SRST_PIN            7
#define TCK_PIN             2
#define TMS_PIN             3
#define TX_PIN              0
#define RX_PIN              1

#define DEBUG_PORT          2
#define DEBUG_TX_PIN        1
#define DEBUG_RX_PIN        0

#define LED_PORT            2
#define DAP_LED_PIN         5
#define MSD_LED_PIN         4
#define CDC_LED_PIN         6

#define BUTTON_PORT         2
#define BUTTON_PIN          7

/* 1-Wire master I/O */
#define OWM_PORT            4
#define OWM_PIN             0

#define EN_VDDIOH_PORT      3
#define EN_VDDIOH_PIN       6

#define SWCLK_PIN           TCK_PIN
#define SWDIO_PIN           TMS_PIN

#define MXC_E_GPIO_OUT_MODE_TRISTATE              0
#define MXC_E_GPIO_OUT_MODE_OPEN_DRAIN_W_PULLUP   2
#define MXC_E_GPIO_OUT_MODE_NORMAL                5

#endif
