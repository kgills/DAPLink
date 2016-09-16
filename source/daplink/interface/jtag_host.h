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
#ifndef JTAGHOST_CM_H
#define JTAGHOST_CM_H

#include "flash_blob.h"
#include "target_reset.h"
#include "debug_cm.h"

#ifdef __cplusplus
extern "C" {
#endif


uint8_t jtag_init(void);
uint8_t jtag_off(void);
uint8_t jtag_read_dp(uint8_t adr, uint32_t *val);
uint8_t jtag_write_dp(uint8_t adr, uint32_t val);
uint8_t jtag_read_ap(uint32_t adr, uint32_t *val);
uint8_t jtag_write_ap(uint32_t adr, uint32_t val);
uint8_t jtag_read_memory(uint32_t address, uint8_t *data, uint32_t size);
uint8_t jtag_write_memory(uint32_t address, uint8_t *data, uint32_t size);
void jtag_set_target_reset(uint8_t asserted);
uint8_t jtag_is_semihost_event(uint32_t *r0, uint32_t *r1);
uint8_t jtag_semihost_restart(uint32_t r0);
uint8_t jtag_flash_syscall_exec(const program_syscall_t *sysCallParam, uint32_t entry, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4);
uint8_t jtag_set_target_state(TARGET_RESET_STATE state);

#ifdef __cplusplus
}
#endif

#endif
