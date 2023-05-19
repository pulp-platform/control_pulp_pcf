
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



#include "tgt_debug.h"

/* FreeRTOS Inclusions. */
#include <FreeRTOS.h>
#include <task.h>

/* Firmware Config */
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_system.h"
#include "cfg_control.h"
#include "cfg_constants.h"

#ifdef CI_TEST
#include "tgt_dbg_ci.h"
#endif

#ifdef MEASURE_ACTIVE
#include "tgt_dbg_measure.h"
#endif

#include "tgt_port.h"
#include "tgt_timer.h"


void vDebugTask(void* parameters) {

	/* Description */
	/* This task is used to print the measure system
	*/
	#ifdef MEASURE_ACTIVE
	Timer_Data_t timer;
	timer.where = 'L';
	#endif

	volatile int wait = 0; //to avoid compiling optimization which deleted code
	int cycles = 0;

	/* Initialization */


	/* Task Code */
	for (;;)
	{

		ulTaskNotifyTake( pdTRUE, 	// xClearCountOnExit: if pdTRUE = Binary Semaphore; if number = Counting Semaphore (# of times to be called before exit).
					portMAX_DELAY); // xTicksToWait


		//TODO: atm, waiting for a better idea:
		#if (defined(CI_TEST) || (MEASURE_ACTIVE == 1))
		vTaskSuspendAll();
		#endif

		#ifdef CI_TEST
			vDebugCiPrintInformation();
		#endif

		#if (MEASURE_ACTIVE==1)
			vDebugMeasurePrintResults();
		#endif

		#ifdef CI_TEST
			vDebugCiExitApplication();
		#endif
		// else
		// {
		// 	//printf("Entering the Else TPM Task\n\r");
		// 	if (CSCheck)
		// 	{
		// 		timerBuffer[Timerindex++] = (Timer_Data_t) {'L', lMeasureReadCsr(MEASURE_ZEROING)};
		// 		CSCheck = 0;
		// 	}
		// }

	/* taskYIELD is used in case of Cooperative Scheduling, to allow/force Contex Switches */
	#if configUSE_PREEMPTION == 0
   		taskYIELD();
	#endif
	}

	/* Cannot and Shouldn't reach this point, but If so... */
	// TODO: SIGNAL HAZARDOUS ERROR, FAILURE!
	vTaskDelete( NULL );
}


void vDebugLaunchDebugTaskFromISR( int now )
{

	//timerBuffer[Timerindex++] = (Timer_Data_t) {'Z', lMeasureReadCsr( MEASURE_ZEROING)};

	//if (now)
    	//vTaskPrioritySet(taskHandles[DEBUG_TASK], (tskIDLE_PRIORITY + 7));

	#if (MEASURE_ACTIVE == 1)
	timerBuffer[Timerindex++] = (Timer_Data_t) {'w', lMeasureReadCsr(MEASURE_ZEROING)};
	#endif

    /* Stop Timer */
    bTargetStopTimer();

    //Last thing to do because it triggers the contex switch automatically
    vTaskNotifyGiveFromISR(taskHandles[DEBUG_TASK], NULL);

    /* Suspend Periodic Task */
    //for (varFor task = 1; task < NUM_PERIODIC_TASKS; task++)
    //    vTaskSuspend(taskHandles[task]);

    portYIELD_FROM_ISR(pdTRUE);

    /* Self Suspend */
    //vTaskSuspend(NULL);
}