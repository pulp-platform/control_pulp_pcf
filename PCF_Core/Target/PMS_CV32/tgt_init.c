
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

#include "tgt_init.h"

/* Target Includes */
#include "system.h"
#include "io.h"
#include "irq.h"
#include "memory_map.h"
#include "timer.h"
#include "timer_hal.h"
#include "timer_irq.h"

/* Libraries Inclusion */
#include "cfg_types.h"

/* pmsis */
#include "cluster/fc_to_cl_delegate.h"
#include "cluster/event_unit.h"
#include "device.h"
#include "target.h"
#include "os.h"

#ifdef PRINTF_ACTIVE
#include <stdio.h>
#include "print_float.h"
#endif


varBool_e bTargetSetVoltage(int iVoltage)
{

    //Nothing to do here
    return PCF_TRUE;
}
varBool_e bTargetSetFrequency(int iFrequency)
{
    //Nothing to do here
    return PCF_TRUE;
}
int lTargetInitializeMemory(void)
{
    //Nothing to do here
    return 0;
}
varBool_e bTargetInitVoltage(void)
{
    //Nothing to do here
    return PCF_TRUE;
}
varBool_e bTargetInitFrequency(void)
{
    //Nothing to do here
    return PCF_TRUE;
}
varBool_e bTargetInitHw(void)
{
    system_init();

#ifdef PCF_USE_CLUSTER

    varBool_e return_value = PCF_TRUE;

    /* Disable printf output buffering to prevent cluster cores clobbering
    * shared buffer. */
    if (setvbuf(stdout, NULL, _IONBF, 0))
    {
        //TODO

        return_value = PCF_FALSE;
    }

    bTargetClusterInit();
#endif

    return PCF_TRUE;
}
