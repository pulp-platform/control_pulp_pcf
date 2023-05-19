
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

#ifndef _PCF_TGT_CLUSTER_H_
#define _PCF_TGT_CLUSTER_H_

/* Libraries Inclusion */
#include "cfg_types.h"

//Global
/* pmsis */
#include "cluster/fc_to_cl_delegate.h"
#include "cluster/event_unit.h"
#include "device.h"
#include "target.h"
#include "os.h"
#include "cluster/cl_team_internal.h"
#include "cluster/event_unit.h"
#include "cluster/cl_idma_hal.h"
struct pi_device cluster_dev;
struct pi_cluster_conf conf;

varBool_e bTargetClusterInit(void);

#ifdef DEBUG_ACTIVE
void vTargetClusterPrintEntry(char* str);
void vTargetClusterPrintExit(char* str);
#endif

varBool_e bTargetClusterSendTaskBlocking(void (*cluster_entry)(void *), void *arg);
varBool_e bTargetClusterSendTaskAsync(void (*cluster_entry)(void *), void *arg, void (*callback)(void *));


#endif //lib #ifdef
