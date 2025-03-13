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

#include "ctrl_math.h"

#include "target.h"
#include "system.h"
#include "timer.h"
#include "timer_hal.h"
#include "timer_irq.h"

#include <math.h>
#include "tgt_cluster.h"

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


#define testt 36




void vTestTask(void *parameters) {

    const varValue ts27_core[27] = {2.74495712,
									2.7491008,
									2.75272832,
									2.7573904,
									2.763096,
									2.76985504,
									2.77768096,
									2.7865904,
									2.79660096,
									2.80773472,
									2.82001504,
									2.8334704,
									2.84813056,
									2.86402912,
									2.88120384,
									2.899696,
									2.91954976,
									2.94081472,
									2.96354592,
									2.98780096,
									3.0136448,
									3.0411472,
									3.0703856,
									3.10144256,
									3.13440992,
									3.16938688,
									3.2064816
	};
	const varValue ts36_core[36] = {2.59944384,
									2.60449744,
									2.60793664,
									2.61235072,
									2.6177488,
									2.62414,
									2.63153808,
									2.63995808,
									2.64941808,
									2.6599384,
									2.6715424,
									2.68425648,
									2.69810976,
									2.7131344,
									2.72936544,
									2.74684384,
									2.7656112,
									2.78571424,
									2.80720576,
									2.83014048,
									2.8545808,
									2.880592,
									2.908248,
									2.93762688,
									2.968816,
									3.00190912,
									3.03700864,
									3.0742288,
									3.1136912,
									3.15553216,
									3.19989952,
									3.24695584,
									3.2968816,
									3.34987456,
									3.40615424,
									3.46596352
	};

	/////
	int num_core = 9; //MAX_NUM_CORE; //27
	struct lFmax_param func_param;
    varValue x_init[testt];

    int max_iterations = 8;
 	varValue min_residual_value = 1e-3;

	//func_param.ptr_wl = (varValue*)malloc(sizeof(varValue)*num_core);
	//func_param.ptr_Pc = (varValue*)malloc(sizeof(varValue)*num_core);
	varValue func_param_wl[testt];
	varValue func_param_Pc[testt];
	func_param.ptr_wl = func_param_wl;
	func_param.ptr_Pc = func_param_Pc;

	//for(int ll=0; ll <5; ll++)
	//for (int l=0; l < 6*2; l++)
	int l,ll=0;
	{
		
		printf("Test %d start\r\n", ll+l);

		uint32_t val_prev, val_after;

		if (l < 6)
		{
			num_core = 9;
		}
		else
		{
			num_core = 9;
		}
		printf("Test with %d cores ", num_core);

		func_param.cores_number = num_core;
		func_param.alpha_vdd = 0.2995; //0.32; //0.2995

		func_param.k_vdd_stat = 0.3243f; //0; //0.3243f
		func_param.k_stat = 0.52804f; //0; //0.52804f

	    for (varFor i = 0; i < num_core; i++)
	 	{
	 		func_param_wl[i] = 1.40516704; //lpower_formula_coeff[1] //TBC:
	 		func_param_Pc[i] = 3.23009760; //reduced_core_power[i]
	 		x_init[i] = 1.5; //i_ctrl_parameter_table->target_freq[i];
	 	}
	 	for (varFor i = num_core; i < testt; i++)
	 	{
	 		func_param_wl[i] = 0; //lpower_formula_coeff[1] //TBC:
	 		func_param_Pc[i] = 0; //reduced_core_power[i]
	 		x_init[i] = 0; //i_ctrl_parameter_table->target_freq[i];
	 	}


	 	// Setup FC timer 0, LO
	    timer_id_t timer0_id = TIMER_LO_ID;       
	    reset_timer_fc(timer0_id);

	 	#ifndef GSL_PARALLEL
	 	vMeasureStop();
		switch (l%6)
		{

			//Here to test better I need also to add performance counters relative to the FPAPU

			case 0:
				vMeasureInit(mCycles);
				printf("and Cycles!!\r\n");
				break;
			case 1:
				vMeasureInit(mInstr);
				printf("and Instruction!!\r\n");
				break;
			case 2:
				vMeasureInit(mLdStall);
				printf("and Load Stall!!\r\n");
				break;
			case 3:
				vMeasureInit(mInstrMiss);
				printf("and Instruction Miss!!\r\n");
				break;
			case 4:
				vMeasureInit(mLoad);
				printf("and Load!!\r\n");
				break;
			case 5:
				vMeasureInit(mStore);
				printf("and Store!!\r\n");
				break;
		}

		vMeasureStart();

	 	switch (l%6)
		{
			case 0:
				val_prev = lMeasureReadCycle(0);
				break;
			case 1:
				val_prev = lMeasureReadInstr(0);
				break;
			case 2:
				val_prev = lMeasureReadLdStall(0);
				break;
			case 3:
				val_prev = lMeasureReadInstrMiss(0);
				break;
			case 4:
				val_prev = lMeasureReadLoad(0);
				break;
			case 5:
				val_prev = lMeasureReadStore(0);
				break;
		}
		#else
	 	struct nwrp_parallel param_parall;
	 	param_parall.ntrial = max_iterations;
		param_parall.x = x_init;
		param_parall.n = num_core; 
		param_parall.tolx = min_residual_value; 
		param_parall.tolf = min_residual_value; 
		param_parall.param = &func_param;
		param_parall.usrfun = usrfunc;
		#endif //GSL_PARALLEL

		start_timer_fc(timer0_id);

		#ifndef GSL_PARALLEL
	 	mnewt(max_iterations, x_init, num_core, min_residual_value, min_residual_value, &func_param, usrfunc);
	 	#else
	 	//bTargetClusterSendTaskBlocking(mnewt_parallel, &param_parall);

	 	struct pi_cluster_task cluster_task;
	    pi_cluster_task(&cluster_task, mnewt_parallel, &param_parall);
	    pi_cluster_send_task_to_cl(&cluster_dev, &cluster_task);
	 	#endif

	 	//stop_timer_fc(timer0_id);
	    uint32_t time0_elapsed = get_timer_cnt_fc(timer0_id);

	 	#ifndef GSL_PARALLEL
	 	switch (l%6)
		{
			case 0:
				val_after = lMeasureReadCycle(0);
				printf("%d, Cycles, %d\r\n", num_core, (val_after - val_prev));
				break;
			case 1:
				val_after = lMeasureReadInstr(0);
				printf("%d, Instr, %d\r\n", num_core, (val_after - val_prev));
				break;
			case 2:
				val_after = lMeasureReadLdStall(0);
				printf("%d, LdStall, %d\r\n", num_core, (val_after - val_prev));
				break;
			case 3:
				val_after = lMeasureReadInstrMiss(0);
				printf("%d, InstMiss, %d\r\n", num_core, (val_after - val_prev));
				break;
			case 4:
				val_after = lMeasureReadLoad(0);
				printf("%d, Load, %d\r\n", num_core, (val_after - val_prev));
				break;
			case 5:
				val_after = lMeasureReadStore(0);
				printf("%d, Store, %d\r\n", num_core, (val_after - val_prev));
				break;
		}
		#endif // GSL_PARALLEL

		//hal_eu_mutex_lock(0);	
	    printf("Timer 0 elapsed: %d\n\r", time0_elapsed);

		if (l < 6)
		{
			//27
			for (varFor i = 0; i < num_core; i++)
			{
				float app = (ts27_core[i] - x_init[i]);
				if (app < 0)
					app = 0.0f-app;
				if (app >= 0.01f)
				{
					printf("Error in NR computation!\r\n");
					for (varFor i = 0; i < testt; i++)
				    {
				        printf("\tx[%d] = ", i);
				        printFloat(x_init[i]);
				        printf("\r\n");
				    }
				    break;
				}
			}
		}
		else
		{
			//36
			for (varFor i = 0; i < num_core; i++)
			{
				float app = (ts36_core[i] - x_init[i]);
				if (app < 0)
					app = 0.0f-app;
				if (app >= 0.01f)
				{
					printf("Error in NR computation!\r\n");
					for (varFor i = 0; i < testt; i++)
				    {
				        printf("\tx[%d] = ", i);
				        printFloat(x_init[i]);
				        printf("\r\n");
				    }
				    break;
				}
			}
		}

		//#endif //GSL_PARALLEL

	}

    for (varFor i = 0; i < testt; i++)
    {
        printf("\tx[%d] = ", i);
        printFloat(x_init[i]);
        printf("\r\n");
    }


	
    /* Task Code */
    for (;;)
    {

        ulTaskNotifyTake( pdTRUE, 	// xClearCountOnExit: if pdTRUE = Binary Semaphore; if number = Counting Semaphore (# of times to be called before exit).
                    portMAX_DELAY); // xTicksToWait

		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'v', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif


		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'V', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif


        /* taskYIELD is used in case of Cooperative Scheduling, to allow/force Contex Switches */
        #if configUSE_PREEMPTION == 0
            taskYIELD();
        #endif
    }

    /* Cannot and Shouldn't reach this point, but If so... */

    // TODO: SIGNAL HAZARDOUS ERROR, FAILURE!
    vTaskDelete( NULL );
}
