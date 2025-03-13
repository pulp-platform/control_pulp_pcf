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

//TODO: remove

//
#ifdef PRINTF_ACTIVE
#include <stdlib.h>
#include <stdio.h>
#include "print_float.h"
#endif

/* Libraries Inclusion */
#include "ctrl_tasks.h"
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_control.h"
#include "cfg_system.h"
#include "cfg_constants.h"

#include "pcf_taskFunctions.h"
#include "sec_functions.h"
#include "ctrl_functions.h"
#include "ctrl_math.h"

//TODO: remove!!!
#include "imp_comms.h"

#ifdef SCMI_ACTIVE
#include "scmi_handler.h"
#endif

#include "tgt_port.h"

/* Others */
#ifdef MEASURE_ACTIVE
#include "tgt_dbg_measure.h"
#endif
#ifdef CI_TEST
#include "tgt_dbg_ci.h"
#endif

#ifdef PCF_USE_CLUSTER
#include "tgt_cluster.h"
//#include "tgt_cluster_hw_def.h"
#include "pmsis_task.h"

SemaphoreHandle_t cl_callback_sem = NULL;
void vClusterCallback(void* arg)
{
	xSemaphoreGiveFromISR(cl_callback_sem, NULL);
}
#endif //PCF_USE_CLUSTER

/*** Tasks ***/

void vPeriodicControlTask(void *parameters ) {

	/* Description */

	/*------------------------------------*/
	/********* Function Variables *********/
	/*------------------------------------*/
	task_param_t l_task_p;
	//TODO put this in presentation
	l_task_p.error_map = BM_RESET;
	l_task_p.task_id = 5397;
	l_task_p.num_cl_cores = 0;

	// Control Variables
	ctrl_values_table_t l_value_ctrl;
	l_task_p.cl_ptr = &l_value_ctrl;

	// Internal to Global Variables	
	//CUGV

	/*------------------------------------*/
	/*********** Initialization ***********/
	/*------------------------------------*/
	//vConfigValueCtrlInit(&l_value_ctrl);
	#if defined(PCF_USE_CLUSTER)
	cl_callback_sem = xSemaphoreCreateBinary();
	#endif

	/* Set PID Parameters */
	if (bMathPidSetParameters(&l_value_ctrl.th) != PCF_TRUE)
	{
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Initialization Error\n\r");
        #endif
        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_task_p.error_map |= BM_ERROR_INITIALIZATION;

        //TODO: do something
        vTargetPcfExit(-3);
    }

    #ifdef PIDV1
	l_value_ctrl.th.kp = 2.0291f;
	l_value_ctrl.th.ki = 420.0093f;
	l_value_ctrl.th.ki *= g_ControlConfigTable.therm.pid_dt;
	l_value_ctrl.th.kd = 0.0f;
	l_value_ctrl.th.kd *= g_ControlConfigTable.therm.pid_dt;
    #elif defined(PIDV2)
	l_value_ctrl.th.kp = 1.9518f;
	l_value_ctrl.th.ki = 73.931f;
	l_value_ctrl.th.ki *= g_ControlConfigTable.therm.pid_dt;
	l_value_ctrl.th.kd = 0.0f;
	l_value_ctrl.th.kd *= g_ControlConfigTable.therm.pid_dt;
    #elif defined(PIDV3)
	l_value_ctrl.th.kp = 1.099f /* *0.75 / 0.1 */ ;
	l_value_ctrl.th.ki = 330.7301f /* *0.5 / 0.1*/;
	l_value_ctrl.th.ki *= g_ControlConfigTable.therm.pid_dt;
	l_value_ctrl.th.kd = 0.0f;
	l_value_ctrl.th.kd *= g_ControlConfigTable.therm.pid_dt;
	#endif
    
	/*------------------------------------*/
	/***** Algorithtm Initialization ******/
	/*------------------------------------*/
	vConfigValueCtrlInit(&l_value_ctrl);

	//xTaskNotifyGive(NULL); //check if null is ok

	/* END Algorithm Initialization */

	/* Task Code */
	for (;;)
	{

		/*-----------------------------------------------*/
		/* 0: Wait the timer to trigger a new execturion */
		/*-----------------------------------------------*/
		ulTaskNotifyTake( pdTRUE,	// xClearCountOnExit: if pdTRUE = Binary Semaphore; if number = Counting Semaphore (# of times to be called before exit).
					portMAX_DELAY); // xTicksToWait. portMAX_DELAY. //TBD


		/*-----------------------------------------------*/
		/********************* Tests *********************/
		/*-----------------------------------------------*/
		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'P',lMeasureReadCsr( MEASURE_ZEROING )};
		#endif

		//
		//printf("periodic: tf: ");
		//printFloat(g_input_ctrl.cmd.target_freq[0]);
		//printf(" - temp: ");
		//printFloat(g_input_ctrl.msr.temp.core[0]);
		//printf(" - og: ");
		//printFloat(l_value_ctrl.ma.og_freq[0]);
		//printf(" - og2: ");

		//ctrl_values_table_t* aa = l_task_p.cl_ptr;
		//printFloat(aa->ma.og_freq[0]);
		//printf("\n\r");

		//Reset
		l_task_p.num_cl_cores = 1;

		//TODO: FIX THISS!!!!
		//ps Do I need this?
		l_value_ctrl.op = g_shared_task.op;

		#ifdef PCF_USE_CLUSTER
		//TBD: here add DMA?	

		#ifndef PCF_USE_CLUSTER_PARALL //THIS is only for INTERNAL TEST ATM, Remove later!
		bTargetClusterSendTaskBlocking(vDummyCluster, &l_task_p);
		#else // !PCF_USE_CLUSTER_PARALL
		bTargetClusterSendTaskAsync(vDummyCluster, &l_task_p, vClusterCallback);
		xSemaphoreTake(cl_callback_sem, portMAX_DELAY);
		#endif // PCF_USE_CLUSTER_PARALL
		#else
		//printf("right . .. . ... \n\r");
		vMainTaskControlAlgorithm(&l_task_p);
		#endif

		//TODO: FIX THISS!!!!
		g_shared_task.op = l_value_ctrl.op;

		//
		//printf("periodic: freq: ");
		//printFloat(l_value_ctrl.op.computed_core_frequency[0]);
		//printf(" - volt: ");
		//printFloat(l_value_ctrl.op.computed_domain_voltage[0]);
		//printf("\n\r");

		/*-----------------------------------------------*/
		/***************** 9: Telemetry  *****************/
		/*-----------------------------------------------*/

		//TODO this part is not right, in no place I add the values.
		//  also, to avoid massive dma usage, I could do telemetry stuff on the FC!
		vTelemetryManagement();

		/* Loop End */
		//TBD: Do we perform a kind of check for this
		//*adr_g_error_map = l_error_map;
		//l_error_map = BM_RESET;

		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'p', lMeasureReadCsr( MEASURE_ZEROING)};
		//printf("\n\n\rcycles: %d\n\r", timerBuffer[Timerindex-3].cycle - timerBuffer[Timerindex-4].cycle);
		#endif

		#ifdef CI_TEST
		ci_test_tasks_exec[PERIODIC_CONTROL_TASK]++;
		#endif

		/* taskYIELD is used in case of Cooperative Scheduling, to allow/force Contex Switches */
		#if configUSE_PREEMPTION == 0
       		taskYIELD();
        #endif
	}

	/* Cannot and Shouldn't reach this point, but If so... */
	// TODO: SIGNAL HAZARDOUS ERROR, FAILURE!
	//TODO l_error_map |= BM_ERROR_REACHED_EOT; //TODO: this has to write the global var directly
	vTaskDelete( NULL );
}

void vRearControlTask(void *parameters ) {

	/* Description */
	/* This task is responsable to be the interpreter and the messenger between the OS of the
	* EPI CPU server and the firmaware of the PMU. The task will also check the voltage regulator
	* values (initially only the EPI core VR, then also HBM, DRAM, MB, and other sockets VR values);
	* it also do an adaptive control algorithm to update the parameters that are used to compute
	* optimal voltage values (both set values and PID parameters).
	* The task will held data as workload histrory, mean power consumption, etc.
	*/

	/* Function Variables */
	varValue MeasuredPowerVR;

	//Least Square Variables
	varValue rms_error;
	varValue theta1[LMS_CN];
	varValue theta2[LMS_CN];
	varValue P1[LMS_CNxCN];
	varValue P2[LMS_CNxCN];
	varValue lms_inputs[LMS_CN];

	uint16_t lms_batch_counter = 0;

	int alternate_flag;

	//Global To Internal Variables
	//telemetry_t l_telemetry;
	//ctrl_inputs_table_t l_ctrl_input_table;
	//telemetry_t l_todo_telemetry;

	varFor l_num_cores = g_config_sys.num_cores;
	uint16_t perc_wl_states[PCF_WL_STATES];

	//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
	uint32_t l_error_map = BM_RESET;
	varError* adr_g_error_map = parameters;

	const uint32_t id = 1367;

	/*** Initialization ***/
	l_error_map = BM_RESET;
	*adr_g_error_map = BM_RESET;

	alternate_flag = 0;
	lms_batch_counter = 0;
	//INITIALIZATION AS IDENTITY
	for (varFor i = 0; i < LMS_CN; i++)
		for (varFor j = 0; j < LMS_CN; j++)
		{
			if (i == j)
			{
				//P1[i*LMS_CN + j] = 1;
				P2[i*LMS_CN + j] = 1;
			}
			else
			{
				//P1[i*LMS_CN + j] = 0;
				P2[i*LMS_CN + j] = 0;
			}
		}

	for (varFor i = 0; i < PCF_WL_STATES; i++)
		for (varFor j = 0; j < PCF_MODEL_COEFF; j++)
		{
			theta2[i*PCF_MODEL_COEFF + j] = g_config_sys.base_power_coeff[i][j];
		}


	/* Task Code */
	for (;;)
	{

		/*-----------------------------------------------*/
		/* 0: Wait the timer to trigger a new execturion */
		/*-----------------------------------------------*/

		ulTaskNotifyTake( pdTRUE, 	// xClearCountOnExit: if pdTRUE = Binary Semaphore; if number = Counting Semaphore (# of times to be called before exit).
					portMAX_DELAY); // xTicksToWait. portMAX_DELAY. //TBD


		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'O', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif

		/*-----------------------------------------------*/
		/********** 1: Read Voltage Regulators ***********/
		/*-----------------------------------------------*/

		/*** 1.B: Read Shared Values ***/ //TBD: after OR BEFORE the ulTaskNotify call?
		//if (bReadGlobalVariable(&l_ctrl_input_table, G_CTRL_INPUT_TABLE, sizeof(l_ctrl_input_table), id) != PCF_TRUE)
		//{
		//	l_error_map |= BM_ERROR_SHARED_VAR_READ;

			//Do Something.. //TODO
		//}


		/*-----------------------------------------------*/
		/********** 2: Check request from EPI OS *********/
		/*-----------------------------------------------*/
		//TODO delete this part below.
		//if (bReadGlobalVariable(&l_todo_telemetry, G_TELEMETRY, sizeof(l_todo_telemetry), id) != PCF_TRUE)
		//{
		//	l_error_map |= BM_ERROR_SHARED_VAR_READ;

			//Do Something.. //TODO
		//}
		//
		/*
		if (bImpReadInputParameters(&l_ctrl_input_table) != PCF_TRUE) //TODO: remove telemetry
		{
			l_error_map |= BM_ERROR_SHARED_VAR_READ;

			//Do Something.. //TODO
		}
		*/

		//printFloat(l_accumulation_telemetry.chip_avg_estimated_power );
		//printFloat(l_todo_telemetry.chip_avg_estimated_power);

		/*-----------------------------------------------*/
		/********** 3: Update Calibration Table **********/
		/*-----------------------------------------------*/
		//TONOTE (todo): qui passo già la misura della cpu, invece che del chip, ma il nome è del chip
		rms_error = 0;
		//TODO: rms_error = (l_todo_telemetry.chip_avg_measured_power - l_todo_telemetry.chip_avg_estimated_power);
		//test_on_number
		//todo
			//configASSERT(abs(rms_error) < AccumulationData.TotalPower)

		//printFloat(rms_error);
		/*
		if ((MeasuredPowerVR - CalibrationTable.TotalPowerBudget ) > 0)
		{
			gAR_emerg = (MeasuredPowerVR - CalibrationTable.TotalPowerBudget); //positive
			printFloat(gAR_emerg);
		}
		else
			gAR_emerg = 0;
		*/

		//TODO: this should not go here imho.
		//lmsCompute(&l_ctrl_input_table);
		//void lmsCompute(ctrl_inputs_table_t *i_ctrl_parameter_table, telemetry_t* i_telemetry_struct)
		//{


			#ifdef LMS_ACTIVE
			//TODO: optimize all
			for (varFor j = 0; j < PCF_WL_STATES; j++)
				perc_wl_states[j] = 0;

			for (varFor i = 0; i < l_num_cores; i++)
			{
				for (varFor j = 0; j < PCF_WL_STATES; j++)
				{
					perc_wl_states[j] += i_ctrl_parameter_table->perc_workload[i][j];
				}
			}

			//printFloat(perc_wl_states[3]);

			//Be Careful! This is TOTALLY MODEL-DEPENDANT!
			//This is not optimized but I prefer to keep it separated from the rest.
			for (varFor i = 0; i < PCF_WL_STATES; i++)
			{
				perc_wl_states[i] /= (varValue)l_num_cores;
				lms_inputs[i*PCF_MODEL_COEFF /*+ 0*/] = perc_wl_states[i] * V_Fixed;
				lms_inputs[i*PCF_MODEL_COEFF + 1] = perc_wl_states[i] * V_Fixed * V_Fixed * i_telemetry_struct->core_avg_sensor_data[i].frequency;
				perc_wl_states[i] = 0.0; //TODO: optimize
				//printFloat(lms_inputs[i*LMS_COEFF_NUMBER /*+ 0*/]);
				//printFloat(lms_inputs[i*LMS_COEFF_NUMBER + 1]);
			}

			if (alternate_flag)
			{
				//TODO: returned value
				lMathLmsRecursive(theta2, P2, theta1, P1, lms_inputs, rms_error, cLMS_LAMBDA);

				if (lms_batch_counter > LMS_BATCH)
				{
					lms_batch_counter++;
					for (int i = 0; i < PCF_WL_STATES; i++)
						for (int j = 0; j < PCF_MODEL_COEFF; j++)
						{
							if (theta2[i*PCF_MODEL_COEFF + j] > cEPI_BASIC_WORKLOAD[i][j] + lms_lim[i][j])
								theta2[i*PCF_MODEL_COEFF + j] = cEPI_BASIC_WORKLOAD[i][j] + lms_lim[i][j];
							else if (theta2[i*PCF_MODEL_COEFF + j] < cEPI_BASIC_WORKLOAD[i][j] - lms_lim[i][j])
								theta2[i*PCF_MODEL_COEFF + j] = cEPI_BASIC_WORKLOAD[i][j] - lms_lim[i][j];
							//i_ctrl_parameter_table->power_formula_coeff[i][j] = theta2[i*PCF_MODEL_COEFF + j];
						}
				}
				else
					lms_batch_counter++;
			}
			else
			{
				//TODO: returned value
				lMathLmsRecursive(theta1, P1, theta2, P2, lms_inputs, rms_error, cLMS_LAMBDA);

				if (lms_batch_counter > LMS_BATCH)
				{
					lms_batch_counter++;
					for (int i = 0; i < PCF_WL_STATES; i++)
						for (int j = 0; j < PCF_MODEL_COEFF; j++)
						{
							if (theta1[i*PCF_MODEL_COEFF + j] > cEPI_BASIC_WORKLOAD[i][j] + lms_lim[i][j])
								theta1[i*PCF_MODEL_COEFF + j] = cEPI_BASIC_WORKLOAD[i][j] + lms_lim[i][j];
							else if (theta1[i*PCF_MODEL_COEFF + j] < cEPI_BASIC_WORKLOAD[i][j] - lms_lim[i][j])
								theta1[i*PCF_MODEL_COEFF + j] = cEPI_BASIC_WORKLOAD[i][j] - lms_lim[i][j];
							i_ctrl_parameter_table->power_formula_coeff[i][j] = theta1[i*PCF_MODEL_COEFF + j];
						}
				}
				else
					lms_batch_counter++;
			}

			alternate_flag = !alternate_flag;

			#if (MEASURE_ACTIVE == 1)
			timerBuffer[Timerindex++] = (Timer_Data_t) {'A', lMeasureReadCsr( MEASURE_ZEROING)};
			#endif

		/*
			if (lms_batch_counter > 410)
			{
				printFloat(rms_error);
			}
		*/
			//

			#else //LMS_ACTIVE //TODO TODO TODO EVERYTHING!!
			//todo
			//not needed for now, but I need to change this all.
			// for (varFor i = 0; i < l_num_cores; i++)
			// {
			// 	//CalibrationTable.Workload[i] = 0;
			//
			// 	for (varFor j = 0; j < PCF_WL_STATES; j++)
			// 	{
			// 		CalibrationTable.Workload[i][j] = recWorkload[i][j];
			// 	}
			//
			// }
			//printFloat(CalibrationTable.Workload[2]);
			/*

			rms_error = rms_error / (varValue)N_EPI_CORE * 1.0;

			if (rms_error > 0)
			{
				for (int i = 0; i < EPI_STATES; i++)
				{
					//printf("baluae\n\r" );
					CalibrationTable.PowerFormulaCoeff[i][0] += rms_error * 0.005;
					if (CalibrationTable.PowerFormulaCoeff[i][0] > app[i][0] + 0.1)
						CalibrationTable.PowerFormulaCoeff[i][0] = app[i][0] + 0.1;
					CalibrationTable.PowerFormulaCoeff[i][1] += rms_error * 0.05;
					if (CalibrationTable.PowerFormulaCoeff[i][1] > app[i][1] + 0.2)
						CalibrationTable.PowerFormulaCoeff[i][1] = app[i][1] + 0.2;
				}
				//printFloat(CalibrationTable.PowerFormulaCoeff[1][1]);
			}
			else //TODO: only else?
			{
				for (int i = 0; i < EPI_STATES; i++)
				{
					//printf("baluae2\n\r" );
					CalibrationTable.PowerFormulaCoeff[i][0] += rms_error * 0.005;
					if (CalibrationTable.PowerFormulaCoeff[i][0] < app[i][0] - 0.1)
						CalibrationTable.PowerFormulaCoeff[i][0] = app[i][0] - 0.1;
					CalibrationTable.PowerFormulaCoeff[i][1] += rms_error * 0.05;
					if (CalibrationTable.PowerFormulaCoeff[i][1] < app[i][1] - 0.2)
						CalibrationTable.PowerFormulaCoeff[i][1] = app[i][1] - 0.2;
				}
				//printFloat(CalibrationTable.PowerFormulaCoeff[1][1]);
			}
		*/
			#endif //LMS_ACTIVE
		//}

		/*-----------------------------------------------*/
		/***************** 4: Write Data *****************/
		/*-----------------------------------------------*/

		/*** 4.1: Write Calibration Table ***/
		/*
		if (bWriteGlobalVariable(&l_ctrl_input_table, G_CTRL_PARAMETER_TABLE, GLOBAL_WRITE_ALL, sizeof(l_ctrl_input_table), id) != PCF_TRUE)
		{
			//TODO: do something
		}

		if (bWriteGlobalVariable(&l_todo_telemetry, G_TELEMETRY, GLOBAL_WRITE_ALL, sizeof(l_todo_telemetry), id) != PCF_TRUE)
		{
			//TODO: do something
		}
		*/

		/*** 4.2: Write Data to the OS Memeory ***/


		/* Loop End */
		//TBD: Do we perform a kind of check for this
		*adr_g_error_map = l_error_map;
		l_error_map = BM_RESET;

		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'o', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif

		#ifdef CI_TEST
		ci_test_tasks_exec[REAR_CONTROL_TASK]++;
		#endif

		/* taskYIELD is used in case of Cooperative Scheduling, to allow/force Contex Switches */
		#if configUSE_PREEMPTION == 0
       		taskYIELD();
    	#endif
	}

	/* Cannot and Shouldn't reach this point, but If so... */
	//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
	l_error_map |= BM_ERROR_REACHED_EOT;
	//#endif
	// TODO: SIGNAL HAZARDOUS ERROR, FAILURE!
	vTaskDelete( NULL );
}


/*************************************************************************/
/*************************************************************************/
/*************************************************************************/


void vFastControlTask (void *parameters) {

	/* Description */
	/*
	*
	*/


    //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
    uint32_t l_error_map = BM_RESET;
    varError* adr_g_error_map = parameters;

    const uint32_t id = 1367;

    // Internal to Global Variables
	//ctrl_values_table_t l_ctrl_values_table;

    /*** Initialization ***/
    *adr_g_error_map = BM_RESET;

    //Get initial Values
	//if (bReadGlobalVariable(&l_ctrl_values_table, G_CTRL_PARAMETER_TABLE, sizeof(l_ctrl_values_table), id) != PCF_TRUE)
	/*{
		l_error_map |= BM_ERROR_SHARED_VAR_READ;
		#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
		printf("bReadGlobalVariable issue, l_ctrl_values_table, id: %d\n\r", id);
		#endif

		//Do Something.. //TODO
	}*/

	#if (MEASURE_ACTIVE == 1)
	timerBuffer[Timerindex++] = (Timer_Data_t) {'F', lMeasureReadCsr( MEASURE_ZEROING)};
	#endif

	//Casting Vote
    uint16_t voltage_checking[PCF_DOMAINS_MAX] = {0}; //TBC

	//TODO: 
	int ratio = (g_TaskConfigTable.tasks_period_us[PERIODIC_CONTROL_TASK] / g_TaskConfigTable.tasks_period_us[FAST_CONTROL_TASK]);
	int counter_times = ratio - 1; //

    /* Task Code */
    for (;;)
    {

        ulTaskNotifyTake( pdTRUE, 	// xClearCountOnExit: if pdTRUE = Binary Semaphore; if number = Counting Semaphore (# of times to be called before exit).
                    portMAX_DELAY); // xTicksToWait

		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'F', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif

		counter_times--;
		if (counter_times <= 0)
		{
			counter_times = ratio;

			//TBD: Do we need an extra layer (or a substitutive one) where instead of this we just check a global var? (be careful with races)
			//l_periodicity_counter = g_TaskConfigTable.tasks_tap_multiplier[REAR_CONTROL_TASK]; //TBD: if we use global variable check, this should be put elsewhere.
			// I need to read this every time before computations.
			//if(bReadGlobalVariable(&l_ctrl_values_table, G_CTRL_PARAMETER_TABLE, sizeof(l_ctrl_values_table), id) != PCF_TRUE)
			/*{
				l_error_map |= BM_ERROR_SHARED_VAR_READ;
				#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
				printf("bReadGlobalVariable issue: l_ctrl_values_table, id: %d\n\r", id);
				#endif

				//Do Something.. //TODO
			}*/

			//printf("fast: freq: ");
			//printFloat(g_shared_task.op.computed_core_frequency[0]);
			//printf(" - volt: ");
			//printFloat(g_shared_task.op.computed_domain_voltage[0]);
			//printf("\n\r");

			//for (varFor i = i_start; i < i_end; i+=i_increment)
			/*
			for (varFor core = 0; core < g_config_sys.num_cores; core+=1)
			{
				varValue freq = g_shared_task.op.computed_core_frequency[core];

				uint16_t domain = g_config_sys.core_domain_id[core];
				varFor index = 0;
				while (index < g_config_sys.num_voltage_levels)
				{
					if (freq <= g_config_sys.voltage_table[index*3+2]) //TODO here just lloking at the min freq, not considering voltage overheads or other changes
					{
						if (index > voltage_checking[domain])
						{
							voltage_checking[domain] = index;
						}	
						break; //index = g_config_sys.num_voltage_levels+1;
					}
					else
						index++;
				}
				//checks
				if (index == g_config_sys.num_voltage_levels)
				{
					//TODO error > max!

					#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
					//printf("Error voltage choice core %d > max\n\r", i);
					#endif

					//TODO: put target at max

					//casting vote:
					voltage_checking[index-1 + domain*g_config_sys.core_domain_id[core]]++;
				}
			}

			//for (varFor i = i_start; i < i_end; i+=i_increment)
			for (varFor domain = 0; domain < g_config_sys.num_domains; domain+=1)
			{
				if (g_config_sys.voltage_table[voltage_checking[domain]*3+0] < g_shared_task.op.computed_domain_voltage[domain])
				{
					/*
					printf("Better domain [%d], og-new: ", domain);
					printFloat(g_shared_task.op.computed_domain_voltage[domain]);
					printf(" - ");
					printFloat(g_config_sys.voltage_table[voltage_checking[domain]*3+0]);
					printf("\n\r");
					*/
					/*
					g_shared_task.op.computed_domain_voltage[domain] = g_config_sys.voltage_table[voltage_checking[domain]*3+0];
				}

				voltage_checking[domain] = 0;

			}
			*/

			if (bSecureSendDomainVoltages(g_shared_task.op.computed_domain_voltage) != PCF_TRUE)
			{
				//TODO

				#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
				printf("Error in sending Voltages\n\r");
				#endif
			}

			//printf("Vdd: ");
			//printFloat(computed_domain_voltage[0]);
			//printf("\n\r");
		}
		#ifdef SCMI_ACTIVE
		//TODO: THIS IS JUST IMPLEMENTED FOR THE PAPER!!! THE CORRECT thiNG IS WAY DIFFERENT
		if (scmi_target_changed && SCMI_IMPROV)
		{
			//printf("improv\n\r");
			scmi_target_changed = 0;
			varValue freq = 0;
			if (scmi_target_frequency == 1) {
				freq = 0.8;
			}else if (scmi_target_frequency == 2) {
				freq = 2.41f;		
			}else if (scmi_target_frequency == 3) {
				freq = 3.4;
			}
			if (freq < g_shared_task.op.computed_core_frequency[SCMI_CORE])
			{
				lowered_freq = 2;

				/*
				printf("lower freq og-new: ");
				printFloat(g_shared_task.op.computed_core_frequency[SCMI_CORE]);
				printf(" - ");
				printFloat(freq);
				printf("\n\r");
				*/
				
				//TODO: HERE I SHOULD CHECK VARIOUS THINGS, I'M JUST ASSUMING 1 DOMAIN PER CORE, THIS IS DONE
				//		JUST FOR THE PAPER!!!!!!!!!
				g_shared_task.op.computed_core_frequency[SCMI_CORE] = freq;
				fast_freq[2] = freq;
				//TODO: ALSO I'M NOT CONSIDERING PLL TIME TO CHANGE, AND I'M NOT CONSIDERING THE FACT THAT I SHOULD LOWER THE FREQUENCY THAN WAIT FOR THE VOLTAGE!
				bSecureSendCoreFrequencies(g_shared_task.op.computed_core_frequency);
				varFor index = 0;
				while (index < g_config_sys.num_voltage_levels)
				{
					if (freq <= g_config_sys.voltage_table[index*3+2]) //TODO here just lloking at the min freq, not considering voltage overheads or other changes
					{
						if (index > voltage_checking[SCMI_CORE])
						{
							voltage_checking[SCMI_CORE] = index;
						}	
						break; //index = g_config_sys.num_voltage_levels+1;
					}
					else
						index++;
				}
				//checks
				if (index == g_config_sys.num_voltage_levels)
				{
					//TODO error > max!

					#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
					//printf("Error voltage choice core %d > max\n\r", i);
					#endif

					//TODO: put target at max

					//casting vote:
					voltage_checking[index-1 + SCMI_CORE*g_config_sys.core_domain_id[SCMI_CORE]]++;
				}

				if (g_config_sys.voltage_table[voltage_checking[SCMI_CORE]*3+0] < g_shared_task.op.computed_domain_voltage[SCMI_CORE])
				{
					/*
					printf("Better domain [%d], og-new: ", SCMI_CORE);
					printFloat(g_shared_task.op.computed_domain_voltage[SCMI_CORE]);
					printf(" - ");
					printFloat(g_config_sys.voltage_table[voltage_checking[SCMI_CORE]*3+0]);
					printf("\n\r");
					*/

					g_shared_task.op.computed_domain_voltage[SCMI_CORE] = g_config_sys.voltage_table[voltage_checking[SCMI_CORE]*3+0];
				}

				if (bSecureSendDomainVoltages(g_shared_task.op.computed_domain_voltage) != PCF_TRUE)
				{
					//TODO

					#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
					printf("Error in sending Voltages\n\r");
					#endif
				}

			}
		}
		#endif
		/* Loop End */
		//TBD: Do we perform a kind of check for this
		*adr_g_error_map = l_error_map;
		l_error_map = BM_RESET;

		#if (MEASURE_ACTIVE == 1)
		timerBuffer[Timerindex++] = (Timer_Data_t) {'f', lMeasureReadCsr( MEASURE_ZEROING)};
		#endif

		#ifdef CI_TEST
		ci_test_tasks_exec[FAST_CONTROL_TASK]++;
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

