
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


/********************************************************/
/*
* File:
* Notes:
*
* Written by: Eventine (UNIBO)
*
*********************************************************/

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



//Global
struct pi_device cluster_dev;
struct pi_cluster_conf conf;

struct pi_cluster_task cluster_task;
//#ifdef PCF_USE_CLUSTER_PARALL
pi_task_t callback_task;
//#endif

//TODO: This library it is not properly done, it is just an hotfix for the cluster:
//  What happens when multiple callback are called? I guess I need to define multiple callback_task?
//  Same thing with cluster_task, what happen if more task are sent to the cluster?
//      HOW they are executed? In Which order?


varBool_e bTargetClusterInit(void)
{
    varBool_e return_value = PCF_TRUE;

    /* First open the cluster */
    pi_cluster_conf_init(&conf);
    pi_open_from_conf(&cluster_dev, &conf);

    //Check.
    if (pi_cluster_open(&cluster_dev))
    {
        //TODO

        #ifdef DEBUG_ACTIVE
        printf("Error! The Cluster is not ready!\n\r");
        #endif

        return_value = PCF_FALSE;
    }

    return return_value;
}

#ifdef DEBUG_ACTIVE
inline void vTargetClusterPrintEntry(char* str)
{
    hal_eu_mutex_lock(0);
    printf("(%ld, %ld) Entering cluster controller from %s\n", pi_cluster_id(), pi_core_id()), str;
    hal_eu_mutex_unlock(0);
}

inline void vTargetClusterPrintExit(char* str)
{
    hal_eu_mutex_lock(0);
    printf("(%ld, %ld) Return from cluster controller to Manager core at %s\n", pi_cluster_id(), pi_core_id(), str);
    hal_eu_mutex_unlock(0);
}
#endif


varBool_e bTargetClusterSendTaskBlocking(void (*cluster_entry)(void *), void *arg)
{
    varBool_e return_value = PCF_TRUE;

    pi_cluster_task(&cluster_task, cluster_entry, arg);
    pi_cluster_send_task_to_cl(&cluster_dev, &cluster_task);

    return return_value;
}


varBool_e bTargetClusterSendTaskAsync(void (*cluster_entry)(void *), void *arg, void (*callback)(void *))
{
    varBool_e return_value = PCF_TRUE;

    pi_cluster_task(&cluster_task, cluster_entry, arg);
    pi_task_callback(&callback_task, callback, (void *)&callback_task);
    pi_cluster_send_task_to_cl_async(&cluster_dev, &cluster_task, &callback_task);

    //__pi_task_destroy(&fc_task);

    return return_value;
}
