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

#ifndef _PCF_TGT_INIT_H_
#define _PCF_TGT_INIT_H_

/* Libraries Inclusion */
#include "cfg_types.h"

varBool_e bTargetSetVoltage(int iVoltage);
varBool_e bTargetSetFrequency(int iFrequency);

// return the total amount of memory setted
int lTargetInitializeMemory(void);

varBool_e bTargetInitVoltage(void);
varBool_e bTargetInitFrequency(void);
varBool_e bTargetInitHw(void);

#endif // lib #ifdef
