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

/* FreeRTOS Inclusions. */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* Libraries Inclusion */
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_system.h"
#include "cfg_control.h"
#include "cfg_constants.h"

#include "sec_globalProtectedVar.h"

#include "imp_comms.h"
#include "ctrl_math.h"
#include "ctrl_tasks.h"
#include "pcf_tasks.h"

/* Others */
#ifdef MEASURE_ACTIVE
#include "tgt_dbg_measure.h"
#endif
#ifdef CI_TEST
#include "tgt_dbg_ci.h"
#endif

#ifdef PRINTF_ACTIVE
#include <stdio.h>
#include "print_float.h"
#endif

/* Others */

void vMainTask(void *parameters) {

    /* Description */
    /*
     *
     */

    // if (g_CodeConfigTable.use_error_map == PCF_TRUE)
    uint32_t l_error_map = BM_RESET;
    varError *adr_g_error_map = parameters;

    const uint32_t id = 1367;

    /*** Initialization ***/
    *adr_g_error_map = BM_RESET;

#if (MEASURE_ACTIVE == 1)
    timerBuffer[Timerindex++] = (Timer_Data_t){'G', lMeasureReadCsr(MEASURE_ZEROING)};
#endif

    /* Task Code */
    for (;;) {

        ulTaskNotifyTake(pdTRUE, // xClearCountOnExit: if pdTRUE = Binary Semaphore; if number =
                                 // Counting Semaphore (# of times to be called before exit).
                         portMAX_DELAY); // xTicksToWait

#if (MEASURE_ACTIVE == 1)
        timerBuffer[Timerindex++] = (Timer_Data_t){'G', lMeasureReadCsr(MEASURE_ZEROING)};
#endif

        /* Do BMC STUFF */

        /*
        if (l_error_map != BM_RESET)
        {
            if(!lWriteErrorMap(ErrorMap))
            {

                //TODO: do something. If the shared var is not working I need to find another way to
        notify the problem.

            }
            else
            {
                l_error_map = BM_RESET;
            }
        }
        */

        /* Loop End */
        // TBD: Do we perform a kind of check for this
        *adr_g_error_map = l_error_map;
        l_error_map = BM_RESET;

#if (MEASURE_ACTIVE == 1)
        timerBuffer[Timerindex++] = (Timer_Data_t){'g', lMeasureReadCsr(MEASURE_ZEROING)};
#endif

#ifdef CI_TEST
        ci_test_tasks_exec[MAIN_TASK]++;
#endif

/* taskYIELD is used in case of Cooperative Scheduling, to allow/force Contex Switches */
#if configUSE_PREEMPTION == 0
        taskYIELD();
#endif
    }

    /* Cannot and Shouldn't reach this point, but If so... */

    l_error_map |= BM_ERROR_REACHED_EOT;

    // TODO: SIGNAL HAZARDOUS ERROR, FAILURE!
    vTaskDelete(NULL);
}
