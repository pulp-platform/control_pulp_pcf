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

#ifndef _CTRL_FUNCTIONS_H_
#define _CTRL_FUNCTIONS_H_

/* Libraries Inclusion */
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_control.h"

// TODO
#include "ctrl_math.h"

// forward declaration
struct ctrl_task_index;

struct function_param {
    // TODO: warning: assignment to 'varValue (*)(varFor,  varFor,  varFor,  void **)'
    //{aka 'float (*)(int,  int,  int,  void **)'} from incompatible pointer type
    //'void (*)(varFor,  varFor,  varFor,  void **)' {aka 'void (*)(int,  int,  int,  void **)'}
    //[-Wincompatible-pointer-types]
    varValue (*f)(varFor, varFor, varFor, struct ctrl_task_index *, void **);
    varFor total_iterations;
    varFor parall_num;
    int exec_time;
    void **ptr_args;
    struct ctrl_task_index *igl;
    int num_args;
    varValue *return_value;
};

struct ctrl_task_index {
    ctrl_config_table_t *config_ctrl;
    config_sys_t *config_sys;
    ctrl_values_table_t *values;
    ctrl_inputs_t *inputs;
    telemetry_t *tel;
    task_param_t *tp;
    struct function_param *fp;
};

void vMainTaskControlInit(task_param_t *tp);
void vMainTaskControlAlgorithm(task_param_t *tp);
void vTelemetryManagement(void);

void vDummyCluster(void *args);

// TODO change name and move position
void vConfigValueCtrlInit(ctrl_values_table_t *values);

// TODO REMOVE:
int abrakazam;

#endif // lib #ifdef
