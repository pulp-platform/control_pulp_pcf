/*************************************************************************
 *
 * Copyright 2023 ETH Zurich and University of Bologna
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
 *
 * SPDX-License-Identifier: Apache-2.0
 * Author: Giovanni Bambini (gv.bambini@gmail.com)
 *
 **************************************************************************/

#ifndef _PCF_SCMI_HANDLER_H_
#define _PCF_SCMI_HANDLER_H_
#ifdef SCMI_ACTIVE

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

/* system includes */
#include "system.h"
#include "io.h"
#include "irq.h"
#include "csr.h"
#include "clic.h"
#include "scmi.h"

/* pmsis */
// #include "target.h"
// #include "os.h"

// TODO:
#define MBOX_START_ADDRESS 0xFFFF0000
#define INTR_ID 32
#define CLIC_BASE_ADDR 0x1A200000
#define CLIC_END_ADDR 0x1A20FFFF
#define SUCCESS 0x0
#define FC_TIMER_BASE_ADDR 0x1B200400
#define ARCHI_TIMER_SIZE 0x00000800
#define FC_TIMER_ADDR (0x1A100000 0x0000B000)
#define DRAM_BASE_ADDR 0x20000000

uint32_t scmi_target_frequency;
// TODO:
uint32_t scmi_target_changed;
#define SCMI_CORE 2

void scmi_init(void);
void enable_scmi_interrupt(void);
void callee_scmi_handler(void);

int lowered_freq;
float fast_freq[4];

#endif // SCMI_ACTIVE
#endif // lib #ifdef
