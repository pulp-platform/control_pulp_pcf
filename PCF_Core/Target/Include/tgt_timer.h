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

#ifndef _PCF_TGT_TIMER_H_
#define _PCF_TGT_TIMER_H_

/* Libraries Inclusion */
#include "cfg_types.h"

varBool_e bTargetSetTimer(uint32_t time_us);
varBool_e bTargetStartTimer(void);
varBool_e bTargetStopTimer(void);

uint32_t usTargetTimerTapTest(uint32_t tap_us);

/* Interrupt Handlers */
void TIMER1_IRQ_handler();

#endif // lib #ifdef
