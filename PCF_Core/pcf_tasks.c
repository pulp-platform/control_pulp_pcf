
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





//TODO: all these inclusions below, to see which are needed I just copy pasted.
/* FreeRTOS Inclusions. */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* Libraries Inclusion */
#include "ctrl_tasks.h"
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_control.h"
#include "cfg_system.h"
#include "cfg_constants.h"
#include "pcf_taskFunctions.h"
#include "imp_comms.h"
#include "ctrl_functions.h"
#include "ctrl_math.h"
#include "ctrl_functions.h"


//#include "tgt_port.h"

#ifdef MEASURE_ACTIVE
#include "tgt_dbg_measure.h"
#endif
#ifdef CI_TEST
#include "tgt_dbg_ci.h"
#endif

/* Others */
//TODO:remove
#include <stdio.h>


void vCommsTask(void *parameters ) {

	/* Description */
	/* This task is responsable of dialoguing with the BMC task of the core of the other socket (same MB)
	* This task will be activated by a notification sent by TControl that will check the necessity of trasmission
	* through a polling (500 ms of period)
	*/

	//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
	uint32_t l_error_map = BM_RESET;
	varError* adr_g_error_map = parameters;
	ctrl_inputs_table_t l_ctrl_input_table;

	//TBC: these variables
	const uint32_t id = 7321;

	*adr_g_error_map = BM_RESET;

	#if (MEASURE_ACTIVE == 1)
	timerBuffer[Timerindex++] = (Timer_Data_t) {'M', lMeasureReadCsr( MEASURE_ZEROING)};
	#endif

	//Get initial Values
	if (bReadGlobalVariable(&l_ctrl_input_table, G_CTRL_INPUT_TABLE, sizeof(l_ctrl_input_table), id) != PCF_TRUE)
	{
		l_error_map |= BM_ERROR_SHARED_VAR_READ;

		//Do Something.. //TODO
	}

	/* Task Code */
	for (;;)
	{
		ulTaskNotifyTake( pdTRUE, 	// xClearCountOnExit: if pdTRUE = Binary Semaphore; if number = Counting Semaphore (# of times to be called before exit).
					portMAX_DELAY); // xTicksToWait

		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'M', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif

		//TODO FIX THIS, THIS IS NEEEDED OTHERWISE THIS TASK WILL OVERRIDE THE VOLTAGE WRITTEN BY THE PERIODIC TASK. IT SHOULD NOT -be NEEDED-
		//fix this by restructuring everything
		/*
		if(bReadGlobalVariable(&l_ctrl_input_table, G_CTRL_INPUT_TABLE, sizeof(l_ctrl_input_table), id) != PCF_TRUE)
		{
			l_error_map |= BM_ERROR_SHARED_VAR_READ;
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			printf("bReadGlobalVariable issue: l_ctrl_input_table, id: %d\n\r", id);
			#endif

			//Do Something.. //TODO
		}

		if (bImpReadInputParameters(&l_ctrl_input_table) != PCF_TRUE) //TODO: remove telemetry
		{
			l_error_map |= BM_ERROR_SHARED_VAR_READ;


			//Do Something.. //TODO
		}

		if (bImpReadInstructionComposition(&l_ctrl_input_table) != PCF_TRUE) //TODO: remove telemetry
		{
			l_error_map |= BM_ERROR_SHARED_VAR_READ;



			//Do Something.. //TODO
		}

		/*** Write Calibration Table ***/
		/*
		if (bWriteGlobalVariable(&l_ctrl_input_table, G_CTRL_INPUT_TABLE, GLOBAL_WRITE_ALL, sizeof(l_ctrl_input_table), id) != PCF_TRUE)
		{
			//TODO: do something

		}
		*/


		/* Loop End */
		//TBD: Do we perform a kind of check for this
		*adr_g_error_map = l_error_map;
		l_error_map = BM_RESET;

		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'m', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif

		#ifdef CI_TEST
		ci_test_tasks_exec[COMMS_TASK]++;
		#endif

		/* taskYIELD is used in case of Cooperative Scheduling, to allow/force Contex Switches */
		#if configUSE_PREEMPTION == 0
	   		taskYIELD();
		#endif
	}

	/* Cannot and Shouldn't reach this point, but If so... */

	l_error_map |= BM_ERROR_REACHED_EOT;

	// TODO: SIGNAL HAZARDOUS ERROR, FAILURE!
	vTaskDelete( NULL );
}

void vUserDefinedTask(void *parameters) {


	//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
	uint32_t l_error_map = BM_RESET;
	varError* adr_g_error_map = parameters;

	//TBC: these variables
	const uint32_t id = 7321;

	*adr_g_error_map = BM_RESET;

	#if (MEASURE_ACTIVE == 1)
	timerBuffer[Timerindex++] = (Timer_Data_t) {'U', lMeasureReadCsr( MEASURE_ZEROING)};
	#endif

    /* Task Code */
    for (;;)
    {

        ulTaskNotifyTake( pdTRUE, 	// xClearCountOnExit: if pdTRUE = Binary Semaphore; if number = Counting Semaphore (# of times to be called before exit).
                    portMAX_DELAY); // xTicksToWait

		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'U', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif

	//lImpUserFunction(NULL);

        /*
        if (l_error_map != BM_RESET)
        {
            if(!lWriteErrorMap(ErrorMap))
            {

                //TODO: do something. If the shared var is not working I need to find another way to notify the problem.

            }
            else
            {
                l_error_map = BM_RESET;
            }
        }
        */

		/* Loop End */
		//TBD: Do we perform a kind of check for this
		*adr_g_error_map = l_error_map;
		l_error_map = BM_RESET;

		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'u', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif

		#ifdef CI_TEST
		ci_test_tasks_exec[USER_DEFINED_TASK]++;
		#endif

        /* taskYIELD is used in case of Cooperative Scheduling, to allow/force Contex Switches */
        #if configUSE_PREEMPTION == 0
            taskYIELD();
        #endif
    }

    /* Cannot and Shouldn't reach this point, but If so... */

    l_error_map |= BM_ERROR_REACHED_EOT;

    // TODO: SIGNAL HAZARDOUS ERROR, FAILURE!
    vTaskDelete( NULL );
}


