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

/* Firmware Config */
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_system.h"
#include "cfg_control.h"
#include "cfg_constants.h"

/* Firmware Libraries */
//

/* Secure */
#include "sec_globalProtectedVar.h"
#include "sec_tasks.h"
#include "sec_functions.h"

/* Target */
#include "tgt_init.h"
#include "tgt_timer.h"
#include "tgt_port.h"
//
#ifdef DEBUG_ACTIVE
#include "tgt_debug.h"
#endif
#ifdef MEASURE_ACTIVE
#include "tgt_dbg_measure.h"
#endif
#ifdef CI_TEST
#include "tgt_dbg_ci.h"
#endif


/* Implementation */
#include "imp_comms.h"

/* Others */
#ifdef PRINTF_ACTIVE
#include <stdio.h>
#include "print_float.h"
#endif
//TODO: DELETE:
#include <stdint.h>

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file.  See https://www.freertos.org/a00016.html */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/* Tasks Management Var */
TaskHandle_t taskHandles[MAX_NUM_TASKS] = {NULL};
//int16_t g_tasks_mul_factor[MAX_NUM_TASKS] = {0}; //signed because counters are signed
/* Interrupt Handlers */
// void TIMER1_IRQ_handler();
//counters need to be signed!
varShortCounter task_period_counter_ISR[NUM_PERIODIC_TASKS] = {0}; //TBD

int lowered_freq = 0;
float fast_freq[4] = {0.8, 0.8, 0.8, 0.8};

/* Periodicity Generator */ //TBD: Move Interrupt Handler to secureSomething or leave it here?
void TIMER1_IRQ_handler()
{
	/* Test */
	#ifdef CI_TEST
	vDebugTimerInterruptTest();
	ci_test_counter[0]++;
	#elif (MEASURE_ACTIVE == 1)
	timerBuffer[Timerindex++] = (Timer_Data_t) {'I', lMeasureReadCsr(MEASURE_ZEROING)};
	#endif


	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t xHigherPriorityTaskWokenToken = pdFALSE;

	for (varFor task = 0; task < NUM_PERIODIC_TASKS; task++)
	{
		task_period_counter_ISR[task]--;

		if (task_period_counter_ISR[task] <= 0) //TODO: if is < (strictly), issue an error
		{
			task_period_counter_ISR[task] = (varShortCounter)g_TaskConfigTable.tasks_tap_multiplier[task];
			vTaskNotifyGiveFromISR( taskHandles[task], &xHigherPriorityTaskWokenToken );

			xHigherPriorityTaskWoken |= xHigherPriorityTaskWokenToken;

		}
	}


	/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	//TBD: Since we want always to trigger the CS, What if we pass true and use the var
	// xHigherPriority... to check if everything is ok?
}


/* Program Entry */
int main( void )
{
	// --------------------------------------------------------------- //
	/*** Initialization  ***/
	// --------------------------------------------------------------- //

	/** Hardware Init **/
	if (bTargetInitVoltage() != PCF_TRUE)
	{
		//TODO: something
		vTargetPcfExit(-2);
	}
	//TODO: should we first check if the Voltage is applied?
	if (bTargetInitFrequency() != PCF_TRUE)
	{
		//TODO: something
		vTargetPcfExit(-2);
	}
	//TBD: Before of After the Voltage/Freq changes?
	if (bTargetInitHw() != PCF_TRUE)
	{
		//TODO: something
		vTargetPcfExit(-2);
	}

	/* To Print the System Clock */
	#if (defined(PRINTF_ACTIVE) && defined(HRO_PRINTF))
	printf("Number of Cores: %d,\n\rSystem Clock: %u Hz\n\r", g_config_sys.num_cores, DEFAULT_SYSTEM_CLOCK); //TODO: put here SystemCoreClock, instead of my definition
	#endif

	/* Hardware Init pt.2 - Comms*/
	if (bImpCommsInit() != PCF_TRUE)
	{
		//TODO: something.
	}

	/* Memory Allocation */
	if (lTargetInitializeMemory() < 0)
	{
		//TODO: something
		vTargetPcfExit(-2);
	}

	//----
	/*** Read Config Values ***/
	//TODO: secure --> should I do from ISR?
	if (bSecureReadConfig() != PCF_TRUE)
	{
		//TODO: something.
	}

	/* Save the internal Original Var */
	//TODO: how to be certain that they are const/not changed.
	//default_SysConfigTable 		= g_config_sys;
	//default_TaskConfigTable		= g_TaskConfigTable;
	//default_CodeConfigTable		= g_CodeConfigTable;
	//default_ControlConfigTable	= g_ControlConfigTable;

	//init global ISR counters
	for (varFor task = 0; task < NUM_PERIODIC_TASKS; task++)
		task_period_counter_ISR[task] = (varShortCounter)g_TaskConfigTable.tasks_tap_multiplier[task];

	/*** FreeRTOS Setup ***/
	if (bSecureFreertosSetup() != PCF_TRUE)
	{
		//TODO: something
		vTargetPcfExit(-4);
	}

	/*** Firmware Setup ***/
	if (bSecureFirmareSetup() != PCF_TRUE)
	{
		//TODO: something
		vTargetPcfExit(-4);
	}

	/*** Control Parameter Setup ***/
	if (bSecureControlParamSetup() != PCF_TRUE)
	{
		//TODO: something
		vTargetPcfExit(-4);
	}

	/*** Tests Setup ***/
	//JO_TBR
	#if (MEASURE_ACTIVE == 1)
	//TODO: here is not configurable from makefile at the moment
	vMeasureInit(mCycles);
	vMeasureStart(); //TODO move this.
	#endif
	#ifdef CI_TEST
	vDebugCiInit();
	#endif

	/** Timer ISR set up **/
	if (bTargetSetTimer(g_TaskConfigTable.tap_period_us) != PCF_TRUE)  // Timer value to be compared here.
	{
		//TODO: something
		vTargetPcfExit(-2);
	}

	/** Check & Start **/
	//CHECKS LIKE num_core <= MAX_NUM_CORE, etc...
	// check periodic tasks < MAX_NUM_TASKS

	#if (defined(PRINTF_ACTIVE) && defined(HRO_PRINTF))
	printf("\n\r\n\r\t *** Starting FreeRTOS Firmware Test *** \n\r\n\r");
	#endif

	/* Start the Scheduler */
	vTaskStartScheduler();
	

	//It should not reach here!
	//TODO: Something

	vTargetPcfExit(-1);

	return -1;
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */

	//TODO
	taskDISABLE_INTERRUPTS();

	//I choose not to include this in HRO/DEBUG cuz it is so important to always show
	#ifdef PRINTF_ACTIVE
	printf( "error: application malloc failed\n\r" );
	#endif
	__asm volatile( "ebreak" );
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */

	//TODO
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */

	//TODO

	//I choose not to include this in HRO/DEBUG cuz it is so important to always show
	#ifdef PRINTF_ACTIVE
	printf("task overflow! task Handler: %x  -  %s\n\r", pxTask, pcTaskName);

	printf("\n\rMinimum unused remaining stack (in words - 32 bits): \n\r");
	for (varFor task = 0; task < MAX_NUM_TASKS; task++)
	{
		printf("	%s:		%d\n\r", pcTaskGetName(taskHandles[task]), (int)uxTaskGetStackHighWaterMark(taskHandles[task]));
	}

	#endif

	taskDISABLE_INTERRUPTS();
	__asm volatile( "ebreak" );

	//for( ;; );
	vTargetPcfExit(1);
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{

	static int pcf_timerActive = 0;

	//TODO: Improve this or make another structural decision
	if (pcf_timerActive == 0)
	{
		if (bTargetStartTimer() != PCF_TRUE)  // Timer value to be compared here.
		{
			//TODO: something
			vTargetPcfExit(-2);
		}
		else
		{
			pcf_timerActive = 1;
		}
		#ifdef CI_TEST
		vMeasureStart();
		#endif

		/** Send some shit out **/

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		BaseType_t xHigherPriorityTaskWokenToken = pdFALSE;

		for (varFor task = 0; task < NUM_PERIODIC_TASKS; task++)
		{
			task_period_counter_ISR[task]--;

			if (task_period_counter_ISR[task] <= 0) //TODO: if is < (strictly), issue an error
			{
				task_period_counter_ISR[task] = (varShortCounter)g_TaskConfigTable.tasks_tap_multiplier[task];
				vTaskNotifyGiveFromISR( taskHandles[task], &xHigherPriorityTaskWokenToken );

				xHigherPriorityTaskWoken |= xHigherPriorityTaskWokenToken;

			}
		}

		/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
	    should be performed to ensure the interrupt returns directly to the highest
	    priority task.  The macro used for this purpose is dependent on the port in
	    use and may be called portEND_SWITCHING_ISR(). */
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

		//TBD: Since we want always to trigger the CS, What if we pass true and use the var
		// xHigherPriority... to check if everything is ok?
	}

	//JO_TBR
	#ifdef CI_TEST
	vDebugCiIterationCount();
	#endif
	#if (MEASURE_ACTIVE == 1)
	timerBuffer[Timerindex++] = (Timer_Data_t) {'K',lMeasureReadCsr( MEASURE_ZEROING )};
	vDebugMeasureIterationCount();
	#endif

	return;
}
/*-----------------------------------------------------------*/
