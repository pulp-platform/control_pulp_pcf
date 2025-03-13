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

#ifndef _PCF_TASK_FUNCT_H_
#define _PCF_TASK_FUNCT_H_


/* Libraries Inclusion */
#include "cfg_types.h"

/* Other */ //TBU
#include <stddef.h> //size_t

//varBool_e bReadGlobalVariable(void* oAddress, pcf_global_var_e i_var_name, size_t i_var_dim, uint32_t i_caller_id);
//varBool_e bWriteGlobalVariable(void* iAddress, pcf_global_var_e i_var_name, global_var_write_cmd_e i_cmd, size_t i_var_dim, uint32_t i_caller_id);

uint32_t* bSecureGetErrorMapAddress(int task_id);



#endif //lib #ifdef
