
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
* Author: Giovanni Bambini (gv.bambini@gmai.com)
*
**************************************************************************/


/********************************************************/
/*
* File:
* Notes:
*
* Written by: Eventine (UNIBO)
*
*********************************************************/

#ifndef _IMP_ADDRESSES_H_
#define _IMP_ADDRESSES_H_


#define IMP_ADR_OUT_FIRST_CORE_FREQ         0x20000000
#define IMP_ADR_OUT_CORE_FREQ_LAST          0x26FFFFFF
#define IMP_ADR_OUT_OTHER                   0x27000000
#define IMP_ADR_OUT_OTHER_LAST              0x2FFFFFFF

#define IMP_ADR_IN_FIRST_CORE_TEMP          0x30000000
#define IMP_ADR_IN_CORE_TEMP_LAST           0x36FFFFFF
#define IMP_ADR_IN_FIRST_CORE_INSTR         0x37000000
#define IMP_ADR_IN_CORE_INSTR_LAST          0x3DFFFFFF
#define IMP_ADR_IN_POWER_CPU                0x3E000000
#define IMP_ADR_IN_POWER_CPU_LAST           0x3EFFFFFF
#define IMP_ADR_IN_OTHER                    0x3F000000
#define IMP_ADR_IN_OTHER_LAST               0x3FFFFFFF

#define IMP_ADR_CMD_FIRST_CORE_FREQ_T       0x40000000
#define IMP_ADR_CMD_CORE_FREQ_T_LAST        0x40FFFFFF
#define IMP_ADR_CMD_POWER_BUDGET            0x41000000
#define IMP_ADR_CMD_POWER_BUDGET_LAST       0x41FFFFFF
#define IMP_ADR_CMD_FIRST_CORE_BINDINGS     0x42000000
#define IMP_ADR_CMD_CORE_BINDINGS_LAST      0x42FFFFFF
#define IMP_ADR_CMD_OTHER                   0x43000000
#define IMP_ADR_CMD_OTHER_LAST              0x43FFFFFF

#define IMP_ADR_COMP_FIRST_CORE_FREQ        0x50000000
#define IMP_ADR_COMP_CORE_FREQ_LAST         0x56FFFFFF

///

//#define IR_APPLY_CORE_THROTT                22
//#define IR_TEMP_REQUEST                     33

#endif //lib #ifdef
