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

#include "tgt_init.h"
#include "tgt_cluster.h"

/* Target Includes */
#include "system.h"

/* Libraries Inclusion */
#include "cfg_types.h"

/* pmsis */
#include "cluster/fc_to_cl_delegate.h"
#include "cluster/event_unit.h"
#include "device.h"
#include "target.h"
#include "os.h"
#include "cluster/cl_team_internal.h"
#include "cluster/event_unit.h"
#include "cluster/cl_idma_hal.h"
//
#include "pmsis_task.h"

// TODO REMOVE:
#include "ctrl_functions.h"

/* Global Cluster Control Var */
// L1_DATA ctrl_parameters_table_t c_parameters_table;
// L1_DATA ctrl_inputs_table_t     c_inputs_table;
// L1_DATA telemetry_t             c_telemetry_table;

// L1_DATA sys_config_table_t 		c_SysConfigTable;
// L1_DATA tasks_config_table_t 	c_TaskConfigTable;
// L1_DATA code_config_table_t 	c_CodeConfigTable;
// L1_DATA ctrl_config_table_t 	c_ControlConfigTable;

// Global
struct pi_device cluster_dev;
struct pi_cluster_conf conf;

struct pi_cluster_task cluster_task;
// #ifdef PCF_USE_CLUSTER_PARALL
pi_task_t callback_task;
// #endif

// TODO: This library it is not properly done, it is just an hotfix for the cluster:
//  What happens when multiple callback are called? I guess I need to define multiple callback_task?
//  Same thing with cluster_task, what happen if more task are sent to the cluster?
//      HOW they are executed? In Which order?

varBool_e bTargetClusterInit(void) {
    varBool_e return_value = PCF_TRUE;

    /* First open the cluster */
    pi_cluster_conf_init(&conf);
    pi_open_from_conf(&cluster_dev, &conf);

    // Check.
    if (pi_cluster_open(&cluster_dev)) {
        // TODO

#ifdef DEBUG_ACTIVE
        printf("Error! The Cluster is not ready!\n\r");
#endif

        return_value = PCF_FALSE;
    }

    return return_value;
}

#ifdef DEBUG_ACTIVE
inline void vTargetClusterPrintEntry(char *str) {
    hal_eu_mutex_lock(0);
    printf("(%ld, %ld) Entering cluster controller from %s\n", pi_cluster_id(), pi_core_id()), str;
    hal_eu_mutex_unlock(0);
}

inline void vTargetClusterPrintExit(char *str) {
    hal_eu_mutex_lock(0);
    printf("(%ld, %ld) Return from cluster controller to Manager core at %s\n", pi_cluster_id(),
           pi_core_id(), str);
    hal_eu_mutex_unlock(0);
}
#endif

varBool_e bTargetClusterSendTaskBlocking(void (*cluster_entry)(void *), void *arg) {
    varBool_e return_value = PCF_TRUE;

    pi_cluster_task(&cluster_task, cluster_entry, arg);
    pi_cluster_send_task_to_cl(&cluster_dev, &cluster_task);

    return return_value;
}

varBool_e bTargetClusterSendTaskAsync(void (*cluster_entry)(void *), void *arg,
                                      void (*callback)(void *)) {
    varBool_e return_value = PCF_TRUE;

    pi_cluster_task(&cluster_task, cluster_entry, arg);
    pi_task_callback(&callback_task, callback, (void *)&callback_task);
    pi_cluster_send_task_to_cl_async(&cluster_dev, &cluster_task, &callback_task);

    //__pi_task_destroy(&fc_task);

    return return_value;
}

varBool_e bTargetClusterFork(void (*func)(void *), void *arg, int parall_num) {
    varBool_e return_value = PCF_TRUE;

    // pi_cl_team_fork(pi_cl_cluster_nb_cores(), func, arg);
    // pi_cl_team_fork(parall_num, func, arg);
    pi_cl_team_fork(0, func, arg);

    return return_value;
}

// TODO
inline void vTargetClusterTeamBarrier(void) {
    pi_cl_team_barrier();
}

void vDummyFork(void *args) {
    struct function_param *param = (struct function_param *)args;

    varFor i = (varFor)pi_cl_team_nb_cores();
    varFor s = (varFor)pi_core_id();

    varFor it_chunk = param->total_iterations / i;
    if ((param->total_iterations % i) != 0) it_chunk += 1;

    varFor e = s + i * it_chunk; //- l_num_cluster_cores;

    // Handle odd core number
    if (e > param->total_iterations) e = param->total_iterations;

    // hal_eu_mutex_lock(0);
    // printf("Entering cluster core: %d / %d with e: %d\n\r", s, i, e);
    // hal_eu_mutex_unlock(0);

    if (param->return_value == NULL) {
        param->f(s, e, i, param->igl, param->ptr_args);

        vTargetClusterTeamBarrier();
    } else {
        param->return_value[s] = param->f(s, e, i, param->igl, param->ptr_args);

        vTargetClusterTeamBarrier();
    }
}
