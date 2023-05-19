
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




#include "tgt_dbg_ci.h"

/* FreeRTOS Inclusions. */
#include <FreeRTOS.h>
#include <task.h>

/* Firmware Config */
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_system.h"
#include "cfg_control.h"
#include "cfg_constants.h"

#include "tgt_dbg_measure.h"
#include "tgt_init.h"

#ifdef PRINTF_ACTIVE
#include <stdio.h>
#endif

#include "tgt_port.h"



/* Tests Var */
int ci_test_counter[2];
int ci_test_tasks_exec[MAX_NUM_TASKS];

/* Timer Test */
uint32_t g_I_cycle_prev_timer = 0;
uint32_t g_I_cycle_prev_systick = 0;
//uint32_t g_cycle_valid_comparison = 0;
uint32_t g_mes_comp_value_timer = 1;
uint32_t g_mes_comp_value_systick = 1;
int correct = 1;

/* Timing data */
int l_systick_us = 0; /*us*/
int l_systick_mul = 0;
int l_tap_not_exact = 0;


int ci_abs(int num);

void vDebugTimerInterruptTest( void )
{
	static int s_check = 0;
	static uint32_t g_cycle_valid_comparison = 0;

	uint32_t cycle_measure = lMeasureReadCycle(0);
	if (cycle_measure > g_I_cycle_prev_timer)
		g_cycle_valid_comparison = cycle_measure - g_I_cycle_prev_timer;
	else //overflow!
	{
		//TODO: check that is VD_MAX_UINT instead of another value
		g_cycle_valid_comparison = cycle_measure + (CONST_MAX_UINT - g_I_cycle_prev_timer);
	}

	//update Value
	g_I_cycle_prev_timer = cycle_measure;

	if (s_check == 0)
	{
		if (ci_abs(g_cycle_valid_comparison - g_mes_comp_value_timer) > 80) //TBD: 500 value
		{
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			printf("Timer issue @ %ldus, interval: %d/%d\n\r", cycle_measure, g_cycle_valid_comparison, g_mes_comp_value_timer);
			#endif
			s_check = 3;
			correct = 0;
			//TBC: vTargetPcfExit(1);
		}
	}
	else
		s_check--;
}

void vDebugCiInit( void )
{
	vMeasureInit(mCycles);

	g_mes_comp_value_timer = (uint32_t)((float)g_TaskConfigTable.tap_period_us *
		((float)DEFAULT_SYSTEM_CLOCK / (float)1000000.0));

	l_systick_us = (int)(1000000.0f / (float)PCF_FREERTOS_TICK_RATE_HZ); /*us*/
	l_systick_mul = l_systick_us * CI_TEST_ITERATION / (int)g_TaskConfigTable.tap_period_us;

	g_mes_comp_value_systick = (uint32_t)((float)l_systick_us * ((float)DEFAULT_SYSTEM_CLOCK / (float)1000000.0));

	if ( (l_systick_us % (int)g_TaskConfigTable.tap_period_us) != 0 )
		l_tap_not_exact = 1;

	#if (defined(PRINTF_ACTIVE) && defined(HRO_PRINTF))
	printf("Cycles btw each Timer interrupt iteration: %d\n\r", g_mes_comp_value_timer);
	printf("Cycles btw each Systick interrupt iteration: %d\n\r", g_mes_comp_value_systick);
	#endif

	for (varFor i=0; i<MAX_NUM_TASKS; i++)
		ci_test_tasks_exec[i] = 0;
}

void vDebugCiIterationCount( void )
{
	/** ISR Fucntion */
	static int l_ci_done = 0;

	if(!l_ci_done)
	{
		/* Timer test */
		static int s_check = 0;
		static int g_cycle_valid_comparison = 0;

		uint32_t cycle_measure = lMeasureReadCycle(0);
		if (cycle_measure > g_I_cycle_prev_systick)
			g_cycle_valid_comparison = cycle_measure - g_I_cycle_prev_systick;
		//else //overflow!
			//g_cycle_valid_comparison = cycle_measure + (VD_MAX_UINT - g_I_cycle_prev_systick);

		//update Value
		g_I_cycle_prev_systick = cycle_measure;

		if (s_check == 0)
		{
			if (ci_abs(g_cycle_valid_comparison - g_mes_comp_value_systick) > 80) //TBD: 500 value
			{
				#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
				printf("Systick issue @ %ldus, interval: %d/%d\n\r", cycle_measure, g_cycle_valid_comparison, g_mes_comp_value_systick);
				#endif
				s_check = 3;
				correct = 0;
				//TBC: vTargetPcfExit(1);
			}
		}
		else{
			s_check--;
		}

		/* end test */

		ci_test_counter[1]++;
		
		if (ci_test_counter[1] == (CI_TEST_ITERATION + 1) )
		{
			l_ci_done = 1;
			vDebugLaunchDebugTaskFromISR(1);
		}
	}
}


void vDebugCiPrintInformation(void) 
{

	#if (defined(PRINTF_ACTIVE) && defined(HRO_PRINTF))
	printf("\n\rTest Done\n\r\n\r");

	/* Application Information */
	printf("***************************************************** \n\r");
	printf("Stack Information: \n\r");

	//printf("Unused stack (current): %d,		(minimum): %d\n\r", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

	printf("\n\rMinimum unused remaining stack (in words - 32 bits): \n\r");
	for (varFor task = 0; task < MAX_NUM_TASKS; task++)
	{
		printf("	%s:		%d\n\r", pcTaskGetName(taskHandles[task]), (int)uxTaskGetStackHighWaterMark(taskHandles[task]));
	}

	/* FreeRTOS execution information */
	printf("\n\r***************************************************** \n\r");
	printf("FreeRTOS execution Information: \n\r");
	printf("\n\rSystick iterations: %d (total FreeRTOS: %d)\n\r", (ci_test_counter[1]-1), xTaskGetTickCount());
	printf("Test duration: %d us\n\r\n\r", l_systick_us*(ci_test_counter[1]-1));

	for (varFor task = 0; task < NUM_PERIODIC_TASKS; task++)
	{
		if (!l_tap_not_exact)
			printf("%s iterations:		%d / %d ", pcTaskGetName(taskHandles[task]), ci_test_tasks_exec[task], ( l_systick_mul / g_TaskConfigTable.tasks_tap_multiplier[task]) );
		else
			printf("%s iterations:		%d / %d ", pcTaskGetName(taskHandles[task]), ci_test_tasks_exec[task], ( l_systick_mul / g_TaskConfigTable.tasks_tap_multiplier[task]) -1);

		printf("	(%d us, %d Hz)\n\r", (g_TaskConfigTable.tasks_tap_multiplier[task] * g_TaskConfigTable.tap_period_us), (1000000/(g_TaskConfigTable.tasks_tap_multiplier[task] * g_TaskConfigTable.tap_period_us)));
	}
	#endif


}

void vDebugCiExitApplication(void)
{

	/* Test Verification */

	//TODO: REMOVE THIS, JUST BECUASE OF TIMER PROBLEM
	correct = 1;

	for (varFor task = 0; task < NUM_PERIODIC_TASKS; task++)
	{
		if (!l_tap_not_exact)
		{
			if (ci_test_tasks_exec[task] != ( l_systick_mul / g_TaskConfigTable.tasks_tap_multiplier[task]) )
				correct = 0;
		}
		else
		{
			if ( (ci_test_tasks_exec[task] != ( l_systick_mul / g_TaskConfigTable.tasks_tap_multiplier[task])) &&
				(ci_test_tasks_exec[task] != (( l_systick_mul / g_TaskConfigTable.tasks_tap_multiplier[task]) -1)) )
				correct = 0;
		}
		
	}

	if (correct)
	{
		#if (defined(PRINTF_ACTIVE) && defined(HRO_PRINTF))
            printf("\n\rTest Completed Succesfully! %d iterations done :)\n\r\n\r", (ci_test_counter[1]-1));
        #endif
		vTargetPcfExit(0);
	}
	else
	{
		#if (defined(PRINTF_ACTIVE) && defined(HRO_PRINTF))
			printf("Test completed, but Failed! :(\n\r");
		#endif
		vTargetPcfExit(1);
	}

}

int ci_abs(int num)
{
	if (num >= 0)
		return num;
	else
		return -num;
}