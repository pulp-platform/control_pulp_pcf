
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


#ifndef _ADDRESSES_H_
#define _ADDRESSES_H_


/*** ADDRESS CONVERSION ***/
#define IMP_ADR_ADDRESS_CONVERTER			0x58000000
#define IMP_ADR_FIRST_ADDRESS				0x20000000
#define IMP_ADR_LAST_ADDRESS				0x2000180F
#define IMP_ADR_OF_CHARACTERS				0xCAFECAFE

/** List **/
#define IMP_ADR_OUT_FIRST_CORE_FREQ         0x20000000
#define IMP_ADR_OUT_CORE_FREQ_LAST          0x20000140
//
#define IMP_ADR_OUT_FIRST_ALPHA             0x20001000
#define IMP_ADR_OUT_ALPHA_LAST	            0x20001140
#define IMP_ADR_OUT_FIRST_REDPW             0x20001150
#define IMP_ADR_OUT_REDPW_LAST	            0x20001290
#define IMP_ADR_OUT_FIRST_FREQREDMAP        0x200012A0
#define IMP_ADR_OUT_FREQREDMAP_LAST         0x200013E0

#define IMP_ADR_IN_FIRST_CORE_TEMP          0x20000150
#define IMP_ADR_IN_CORE_TEMP_LAST           0x20000290
#define IMP_ADR_IN_FIRST_CORE_INSTR         0x200002A0
#define IMP_ADR_IN_CORE_INSTR_LAST          0x200008E0
#define IMP_ADR_IN_POWER_CPU                0x20000900
#define IMP_ADR_IN_POWER_CPU_LAST           0x20000A40

#define IMP_ADR_CMD_FIRST_CORE_FREQ_T       0x20000A50
#define IMP_ADR_CMD_CORE_FREQ_T_LAST        0x20000B90
#define IMP_ADR_CMD_POWER_BUDGET            0x20000BA0
#define IMP_ADR_CMD_POWER_BUDGET_LAST       0x20000CE0
#define IMP_ADR_CMD_FIRST_CORE_BINDINGS     0x20000D00
#define IMP_ADR_CMD_CORE_BINDINGS_LAST      0x20000E40

#define IMP_ADR_COMP_FIRST_CORE_FREQ        NULL
#define IMP_ADR_COMP_CORE_FREQ_LAST         NULL

///

#define IMP_ADR_OUT_FIRST_QUAD_FREQ 		0x20000E50
#define IMP_ADR_OUT_QUAD_FREQ_LAST	 		0x20000ED0
#define IMP_ADR_OUT_FIRST_QUAD_VDD 			0x20000EE0
#define IMP_ADR_OUT_QUAD_VDD_LAST 			0x20000FFF

//#define IR_APPLY_CORE_THROTT                22
//#define IR_TEMP_REQUEST  

//
#define IMP_ADR_RUN_SIMULATION				0x20001800
#define IMP_ADR_PAUSE_SIMULATION			0x20001804
#define IMP_ADR_WORKLOAD_READ				0x20001808

#endif //lib
