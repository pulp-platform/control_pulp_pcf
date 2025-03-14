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

#ifndef _PCF_IMP_COMMS_H_
#define _PCF_IMP_COMMS_H_

/* Libraries Inclusion */
#include "cfg_types.h"
#include "cfg_control.h"

// todo remove
#include "cfg_firmware.h"

varBool_e bImpCommsInit(void);
varBool_e bImpSendCoreFreq(varValue *i_computed_freq);
varBool_e bImpSendDomainVoltages(varValue *i_computed_voltage);
varBool_e bImpReadTempRequest(varValue *o_measured_temp);
// varBool_e bImpReadCoreTemp(varValue *o_measured_temp);

varBool_e bImpReadInputParameters(struct ctrl_commands *i_table_ptr);

varBool_e bImpReadInstructionComposition(struct performance_measures *i_table_ptr);
varBool_e bImpReadPowerMeasure(struct power_measures *i_table_ptr);

// TODO: remove this probably.
varBool_e bImpWriteFreqRedMap(telemetry_t *i_telemetry);

int lImpUserFunction(void *ptr);

#endif // lib #ifdef
