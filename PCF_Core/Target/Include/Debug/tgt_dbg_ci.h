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

#ifndef _PCF_TGT_CI_H_
#define _PCF_TGT_CI_H_

/* FreeRTOS Inclusions. */
#include <FreeRTOS.h>
#include <task.h>

/* Firmware Config */
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_system.h"
#include "cfg_control.h"
#include "cfg_constants.h"

/* Tests Var */
extern int ci_test_counter[2];
extern int ci_test_tasks_exec[MAX_NUM_TASKS];

/* Timer Test */
extern uint32_t g_I_cycle_prev;
extern uint32_t g_cycle_valid_comparison;
extern uint32_t g_mes_comp_value;

void vDebugTimerInterruptTest(void);
void vDebugCiInit(void);
void vDebugCiIterationCount(void);
void vDebugCiPrintInformation(void);
void vDebugCiExitApplication(void);

#endif /* lib */
