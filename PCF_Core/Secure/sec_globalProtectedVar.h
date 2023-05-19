
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
* File: cfg_firmware.h
* Notes: This files holds the definitions, structs and
*           global var regarding the firmware structure and
*           code.
*
* Written by: Eventine (UNIBO)
*
*********************************************************/

#ifndef _PCF_GLOBAL_PROTECTED_VAR_H_
#define _PCF_GLOBAL_PROTECTED_VAR_H_

/* FreeRTOS Inclusions. */
#include <FreeRTOS.h>
#include <semphr.h>

/* Libraries Inclusion */
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_system.h"
#include "cfg_control.h"


extern sys_config_table_t 		default_SysConfigTable;
extern tasks_config_table_t 	default_TaskConfigTable;
extern code_config_table_t 	    default_CodeConfigTable;
extern ctrl_lms_config_table_t  default_LmsConfigTable;
extern ctrl_config_table_t 	    default_ControlConfigTable;


//these has to stay here
extern ctrl_inputs_table_t  g_TasksCtrlInput;
extern ctrl_parameters_table_t g_TasksCtrlParameter;
extern telemetry_t              g_ChipTelemetry;

/* One Mutex for each Shared Global Variable*/
extern SemaphoreHandle_t g_Sem_ChipTelemetry;
extern SemaphoreHandle_t g_Sem_CtrlParameterTable;
extern SemaphoreHandle_t g_Sem_CtrlInputTable;
//extern SemaphoreHandle_t g_SemReductionMap;
//extern SemaphoreHandle_t g_SemErrorMap;

extern uint32_t g_error_map[MAX_NUM_TASKS];


#endif //lib #ifdef
