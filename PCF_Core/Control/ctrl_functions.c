
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




/* Libraries Inclusion */
#include "ctrl_functions.h"

#include "cfg_types.h"
#include "cfg_system.h"
#include "cfg_control.h"
#include "cfg_constants.h"
#include "cfg_firmware.h"

#include "ctrl_math.h"

//TODO
#include "pcf_taskFunctions.h"
#include "sec_functions.h"

//TODO: remove!!!
#include "imp_comms.h"
#include "imp_addresses.h"

//Others
#include <stdint.h>
#ifdef PRINTF_ACTIVE
#include <stdlib.h>
#include <stdio.h>
#include "print_float.h"
#endif
#ifdef MEASURE_ACTIVE
#include "tgt_dbg_measure.h"
#endif


/* Definitions and Constants */
#if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
#define ALPHA_MIN_INIT 1.1f
#else
#define ALPHA_MIN_INIT 2
#endif


/* Global Vars */
ctrl_parameters_table_t c_parameters_table;
ctrl_inputs_table_t c_inputs_table;
telemetry_t c_telemetry_table;
uint32_t c_error_map;
const uint32_t c_task_id = 5367;

/*** Internal Functions ***/
void vCheckFrequencies (varFor i_start, varFor i_end, varFor i_increment);
void vWorkloadProcess(varFor i_start, varFor i_end, varFor i_increment);
void vPowerBudgetProcess(void);
void vPowerConsumptionAdaptation(void);
varValue fComputeEstimatedTotalPower (varFor i_start, varFor i_end, varFor i_increment);
varValue fCorePowerComputation(varFor i_start, varFor i_end, varFor i_increment);
void vVoltageCastVote(varFor i_start, varFor i_end, varFor i_increment, uint16_t* voltage_voting);
void vVoltageChoice(varFor i_start, varFor i_end, varFor i_increment, uint16_t* voltage_voting);
void vDummyPowerReduction(varFor i_start, varFor i_end, varFor i_increment, varValue delta, varValue tot_power);
void vAlphaPowerReduction(varValue i_delta_power);
varValue fComputeAlpha(varFor i_start, varFor i_end, varFor i_increment, varValue delta_power, varValue* Alpha);
void vComputeAlphaBindings(void);
void vNormalizeAlpha(varFor i_start, varFor i_end, varFor i_increment, varValue* Alpha, varValue i_accumulation_Alpha);
void vApplyAlphaReduction(varFor i_start, varFor i_end, varFor i_increment, varValue* Alpha, varValue i_delta_power);
void vComputeTemperatureReduction(varFor i_start, varFor i_end, varFor i_increment);
void vComputeFrequency(void);
void vFrequencyCouplingTransformation(varFor i_start, varFor i_end, varFor i_increment,
                                    int i_max_iterations, varValue i_tolx, varValue i_tolf );
varFor lPopulateCouplingStructure(varFor i_start, varFor i_end, varFor i_increment, varFor i_domain, struct coupling_p2f *i_func_param);
void vProcessFrequency(varFor i_core, varValue i_frequency);
void vFrequency2PowerConversion(varFor i_start, varFor i_end, varFor i_increment);

//TODO:
#define PW_CHANGED_ALPHA_COUNTER 36
#define TF_CHANGED_ALPHA_COUNTER 16

void vMainTaskControlInit (void)
{
	c_error_map = BM_RESET;

	//Get initial Values
	if (bReadGlobalVariable(&c_parameters_table, 
			G_CTRL_PARAMETER_TABLE, sizeof(c_parameters_table), 
			c_task_id) != PCF_TRUE)
	{
		c_error_map |= BM_ERROR_SHARED_VAR_READ;
		#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
		printf("bReadGlobalVariable issue, c_parameters_table, id: %d\n\r", c_task_id);
		#endif

		//Do Something.. //TODO
	}

	if(bReadGlobalVariable(&c_inputs_table, G_CTRL_INPUT_TABLE, sizeof(c_inputs_table), c_task_id) != PCF_TRUE)
	{
		c_error_map |= BM_ERROR_SHARED_VAR_READ;
		#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
		printf("bReadGlobalVariable issue: c_inputs_table, id: %d\n\r", c_task_id);
		#endif

		//Do Something.. //TODO
	}

	//Initialization:

	//ctrl_parameters_table_t
    c_parameters_table.telemetry_counter = c_inputs_table.telemetry_horizon;

	for (varFor core = 0; core < g_SysConfigTable.num_core; core++)
	{
		for (varFor state = 0; state < g_SysConfigTable.core_wl_states_num; state++) 
        {
            c_parameters_table.processed_workload[core][state] = 1.0f/g_SysConfigTable.core_wl_states_num;
        }
	}

	c_parameters_table.prev_power_budget = c_inputs_table.total_power_budget;
	c_parameters_table.power_budget_changed = PCF_FALSE;
    c_parameters_table.total_power_adaptation_term = 0;

    //c_parameters_table.vv_moving_average_alpha = g_ControlConfigTable.vv_moving_average_alpha;
    c_parameters_table.pw_changed_alpha_counter = PW_CHANGED_ALPHA_COUNTER; //init this to be flexible 
	for (varFor core = 0; core < g_SysConfigTable.num_core; core++)
	{
		c_parameters_table.vv_moving_average_og_freq[core]		= g_SysConfigTable.init_core_freq;
		c_parameters_table.vv_moving_average_freq_point[core]	= g_SysConfigTable.init_core_freq;
	}

	for (varFor core = 0; core < g_SysConfigTable.num_core; core++)
	{
		c_parameters_table.ctrl_cmd[core] 				= VD_ZERO;
		c_parameters_table.hyst_thresh_reached[core]	= PCF_FALSE;
		c_parameters_table.measured_temperature[core]	= VD_TEMP_INIT;
	}

	//TODO:
	for (varFor core = 0; core < g_SysConfigTable.num_core; core++)
	{
		c_parameters_table.store_max_freq[core]				= 
			g_SysConfigTable.voltage_table[(0)*3 + 2];
		c_parameters_table.computed_domain_voltage[core]	= 
			g_SysConfigTable.voltage_table[(0)*3 + 0];
		c_parameters_table.computed_core_frequency[core]	= g_SysConfigTable.init_core_freq;
		c_parameters_table.target_core_power[core]			= 1.0; //TODO
	}

	for (varFor i = 0; i < g_SysConfigTable.num_core; i++)
	{
		c_telemetry_table.core_avg_estimated_power[i] 		= g_SysConfigTable.core_idle_power;

		c_telemetry_table.core_avg_sensor_data[i].frequency	= g_SysConfigTable.init_core_freq;
		c_telemetry_table.core_avg_sensor_data[i].voltage 	= g_SysConfigTable.init_core_volt;
		c_telemetry_table.core_avg_sensor_data[i].temperature = VD_TEMP_INIT;
		c_telemetry_table.frequency_reduction_map[i]			= BM_RESET;
		//
	}
	for (varFor i = 0; i < g_SysConfigTable.num_pw_domains; i++)
	{
		c_telemetry_table.quad_avg_estimated_power[i]		= 0;
	}
	c_telemetry_table.chip_avg_estimated_power				= 0;
	c_telemetry_table.chip_avg_measured_power 				= 0;
	c_telemetry_table.power_budget_exceed_us				= 0;

	#if (defined(PCF_USE_CLUSTER) && defined(PCF_USE_CLUSTER_PARALL))
	callback_sem = xSemaphoreCreateBinary();
	#endif

}

void vMainTaskControlAlgorithm (void)
{
	/*-----------------------------------------------*/
	/********* 1: Write Computed Values(k-1) *********/
	/*-----------------------------------------------*/

	//printf("F0: ");
	//printFloat(c_parameters_table.computed_core_frequency[0]);
	//printf("\n\r");

	//Check Freq:
	vCheckFrequencies(0, g_SysConfigTable.num_core, 1);

	//printf("F1: ");
	//printFloat(c_parameters_table.computed_core_frequency[0]);
	//printf("\n\r");

	//send frequencies
	if (bSecureSendCoreFrequencies(c_parameters_table.computed_core_frequency) != PCF_TRUE)
	{
		//TODO

		#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
		printf("Error in sending Frequencies\n\r");
		#endif
	}

	#if (MEASURE_ACTIVE == 1)
	timerBuffer[Timerindex++] = (Timer_Data_t) {'s', lMeasureReadCsr( MEASURE_ZEROING)};
	#endif

	/*-----------------------------------------------*/
	/*************** 2: Read Values(k) ***************/
	/*-----------------------------------------------*/

	if (bSecureReadCoreTemperatures(c_parameters_table.measured_temperature) != PCF_TRUE)
	{
		//TODO

		#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
		printf("Error in reading Temperatures\n\r");
		#endif
	}

	//printf("Temp: ");
	//printFloat(c_parameters_table.measured_temperature[0]);
	//printf("\n\r");

	#if (MEASURE_ACTIVE == 1)
	timerBuffer[Timerindex++] = (Timer_Data_t) {'r', lMeasureReadCsr( MEASURE_ZEROING)};
	#endif

	/*-----------------------------------------------*/
	/**************** 3: Read DATA(k) ****************/
	/*-----------------------------------------------*/
	if (bImpReadInputParameters(&c_inputs_table) != PCF_TRUE) //TODO: remove telemetry
	{
		//l_error_map |= BM_ERROR_SHARED_VAR_READ;


		//Do Something.. //TODO
	}


	if (bImpReadInstructionComposition(&c_inputs_table) != PCF_TRUE) //TODO: remove telemetry
	{
		//l_error_map |= BM_ERROR_SHARED_VAR_READ;



		//Do Something.. //TODO
	}
/*
	if (bWriteGlobalVariable(&c_inputs_table, G_CTRL_INPUT_TABLE, GLOBAL_WRITE_ALL, sizeof(c_inputs_table), c_task_id) != PCF_TRUE)
		{
			//TODO: do something

		}
	*/
/*
	if(bReadGlobalVariable(&c_inputs_table, G_CTRL_INPUT_TABLE, sizeof(c_inputs_table), c_task_id) != PCF_TRUE)
	{
		c_error_map |= BM_ERROR_SHARED_VAR_READ;
		#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
		printf("bReadGlobalVariable issue: c_inputs_table, id: %d\n\r", c_task_id);
		#endif

		//Do Something.. //TODO
	}
*/
	//printf("Tf1: ");
	//printFloat(c_inputs_table.target_freq[0]);
	//printf("\n\r");

	/*-----------------------------------------------*/
	/************** 4: DATA(k) analysis **************/
	/*-----------------------------------------------*/
	
	vWorkloadProcess(0, g_SysConfigTable.num_core, 1);

	/* Check if power budget has changed */
	vPowerBudgetProcess();

	/* Power Model Estimation */
	vPowerConsumptionAdaptation();

	/*-----------------------------------------------*/
	/************ 5: Compute Power Model *************/
	/*-----------------------------------------------*/

    /* Choosing the Voltage */
    //TODO can I unite the two parts? while I casting votes I increase the accumulator, and I stop casting votes when I reached the value? 
	//actually not, I cannot

	//TBD: question, is it better to put the appropriate voltage level or still cast vote for the domain?
	// probably casting votes.
	//Rememeber!! This part has to be REAL too, because total power may be < budget and so, don't go under alpha red
	//still NR will apply
	//but based on a wrong Pc (if not done correctly), so it will be sub-optimal

	//Casting Vote
    uint16_t voltage_voting[MAX_NUM_VOLTAGE_LEVELS*SYS_PW_DOMAIN_NUM] = {0}; //TBC

	// Casting votes to populate voltage array level 
    //TODO: to improve parallelization I could divide by domain and add a if domain == id_domain.
	vVoltageCastVote(0, g_SysConfigTable.num_core, 1, voltage_voting);

	// Deciding the Voltage
	//uint32_t numdomain[SYS_PW_DOMAIN_NUM]={9, 9,9,9}; //TODO: This is hardwired, modify it.
    vVoltageChoice(0, g_SysConfigTable.num_pw_domains, 1, voltage_voting);

	//printf("V1: ");
	//printFloat(c_parameters_table.computed_domain_voltage[0]);
	//printf("\n\r");

    /*** Compute Target Power per Core ***/
    //TODO split for cluster need fix
	varValue estimated_total_power = fCorePowerComputation(0, g_SysConfigTable.num_core, 1);

	//test_on_number
	if (estimated_total_power <= 0)
	{
		c_error_map |= BM_ERROR_NUMBERS_VALUE;
		#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
		//printf("Number issue, estimated_t otal_power: %f\n\r", estimated_total_power);
		printf("Number issue, estimated_total_power: ");
		printFloat(estimated_total_power);
		printf("\n\r");
		#endif

        //TBD:
		estimated_total_power = g_SysConfigTable.core_idle_power * (varValue)g_SysConfigTable.num_core;
		for (varFor i = 0; i < g_SysConfigTable.num_core; i++)
		{
			c_parameters_table.target_core_power[i] = g_SysConfigTable.core_idle_power;
		}
	}

	//printf("P1: ");
	//printFloat(c_parameters_table.target_core_power[0]);
	//printf("\n\r");

	#ifdef USE_CTRL_STRCT_INVERSE
	//reduce Temperature:
	vComputeTemperatureReduction(0, g_SysConfigTable.num_core, 1);
	estimated_total_power = 0;
	for (varFor i = 0; i < g_SysConfigTable.num_core; i+=1)
	{
		estimated_total_power += c_parameters_table.target_core_power[i];
	}
	#endif //USE_CTRL_STRCT_INVERSE

	#ifdef USE_CTRL_STRCT_VB
	varValue reduced_core_power[MAX_NUM_CORE];
	varValue og_core_power[MAX_NUM_CORE];
	for (varFor i=0; i<g_SysConfigTable.num_core; i++)
	{
		og_core_power[i] = c_parameters_table.target_core_power[i];
		//reduced_core_power[i] = c_parameters_table.taget_core_power[i];
	}
	#endif

	/*** Compute Delta Power ***/
    //TODO: domains budgets
	varValue delta_power = estimated_total_power - c_inputs_table.total_power_budget 
        + c_parameters_table.total_power_adaptation_term ;

	/*-----------------------------------------------*/
	/**************** 6: Reduce Power ****************/
	/*-----------------------------------------------*/

	if  (delta_power > 0)
	{
		vAlphaPowerReduction(delta_power);
		//vDummyPowerReduction(0, g_SysConfigTable.num_core, 1, delta_power, (estimated_total_power));
	}

	//printf("P2: ");
	//printFloat(c_parameters_table.target_core_power[0]);
	//printf("\n\r");

	/*-----------------------------------------------*/
	/************* 7: Reduce Temperature *************/
	/*-----------------------------------------------*/

	#ifdef USE_CTRL_STRCT_VB
	for (varFor i=0; i<g_SysConfigTable.num_core; i++)
	{
		reduced_core_power[i] = c_parameters_table.target_core_power[i];
		c_parameters_table.target_core_power[i] = og_core_power[i];
	}	
	#endif
	
	#ifndef USE_CTRL_STRCT_INVERSE
	vComputeTemperatureReduction(0, g_SysConfigTable.num_core, 1);
	#endif


	#ifdef USE_CTRL_STRCT_VB
	for (varFor i=0; i<g_SysConfigTable.num_core;i++)
	{
		if (reduced_core_power[i] < c_parameters_table.target_core_power[i])
		{
			c_parameters_table.target_core_power[i] = reduced_core_power[i];
		}
	}
	#endif

	//printf("P3: ");
	//printFloat(c_parameters_table.target_core_power[0]);
	//printf("\n\r");

	/*-----------------------------------------------*/
	/************** 8: Compute Frequency *************/
	/*-----------------------------------------------*/
	
	vComputeFrequency();

	/*
	printf("F2: ");
	printFloat(c_parameters_table.computed_core_frequency[0]);
	printf("\n\r");
	*/

	//printf("Tf2: ");
	//printFloat(c_inputs_table.target_freq[0]);
	//printf("\n\r");

	#if (MEASURE_ACTIVE == 1)
	timerBuffer[Timerindex++] = (Timer_Data_t) {'C', lMeasureReadCsr( MEASURE_ZEROING)};
	#endif

	//TODO: fix this!!!!1
	//for (varFor domain = 0; domain < g_SysConfigTable.num_pw_domains; domain++)
	//{
	//	c_parameters_table.voltage[domain] = computed_domain_voltage[domain];
	//}

    //TODO: fix and optimize this
	if (bWriteGlobalVariable(&c_parameters_table, G_CTRL_PARAMETER_TABLE, GLOBAL_WRITE_ALL, sizeof(c_parameters_table), c_task_id) != PCF_TRUE)
	{
		//TODO

		#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
		printf("bWriteGlobalVariable issue: l_ctrl_inputs_table, id: %d\n\r", c_task_id);
		#endif
	}
}


void vCheckFrequencies (varFor i_start, varFor i_end, varFor i_increment)
{
	//Check Freq:
	for (varFor i = i_start; i < i_end; i+=i_increment)
	{
		if (c_parameters_table.computed_core_frequency[i] > 
            c_parameters_table.store_max_freq[g_SysConfigTable.core_domain_id[i]]) 
        //g_SysConfigTable.voltage_table[c_parameters_table.computed_domain_voltage[g_SysConfigTable.core_domain_id[i]]*3+2])
		{
/*
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			printf("[%d]: - OGF: ", i);
			printFloat(c_parameters_table.computed_core_frequency[i]);
			printf(" - NF: ");
			printFloat( c_parameters_table.store_max_freq[g_SysConfigTable.core_domain_id[i]]);
			printf("\n\r");
#endif
*/
			c_parameters_table.computed_core_frequency[i] = 
                c_parameters_table.store_max_freq[g_SysConfigTable.core_domain_id[i]];
			c_telemetry_table.frequency_reduction_map[i] |= BM_FRM_VDD_MAX_FREQ_LIMIT;
		}
	}
}

void vWorkloadProcess(varFor i_start, varFor i_end, varFor i_increment)
{
	for (varFor core = i_start; core < i_end; core+=i_increment)
		for (varFor state = 0; state < g_SysConfigTable.core_wl_states_num; state++)
		{
			c_parameters_table.processed_workload[core][state] *= (1.0f-0.4f);
            #ifdef USE_INSTRUCTIONS_COMPOSITION
			c_parameters_table.processed_workload[core][state] += 0.4f*(varValue)c_inputs_table.perc_workload[core][state];
            #else
            //TODO
            #endif
		}
}

void vPowerBudgetProcess(void)
{
	if (c_parameters_table.prev_power_budget != c_inputs_table.total_power_budget)
	{
        //if (g_ControlConfigTable.variable_voltage_solution == VV_MOVING_AVERAGE)
        //if (c_parameters_table.prev_power_budget  < c_inputs_table.total_power_budget) //TBC: only <, because this fix working good on increasing the power budget, but bad on decreasing it.
	//	{
			c_parameters_table.pw_changed_alpha_counter = PW_CHANGED_ALPHA_COUNTER; //16
			//c_parameters_table.vv_moving_average_alpha = 0.8f; //TODO: define a global value: //0.4

			//zero all the things
			/*
			//This is bad, cuz it make Voltage to jump at max value. Instead I should modulate alpha to make it change quicklier
			for (varFor i = 0; i < g_SysConfigTable.num_core; i++)
			{
				c_parameters_table.freq_diff_red[i] = 0;
			}
			*/
	//	}
        //}

		c_parameters_table.power_budget_changed = PCF_TRUE; //TODO move to imp
		c_parameters_table.prev_power_budget = c_inputs_table.total_power_budget;
	}
	else
	{
		c_parameters_table.power_budget_changed = PCF_FALSE;
	}
    //}

}

void vPowerConsumptionAdaptation(void)
{
    //TODO MOVE THIS INTO FAST TASK
	bImpReadPowerMeasure(&c_inputs_table);

    //TODO split for cluster need fix
	varValue estimated_reduced_total_power = fComputeEstimatedTotalPower(0, g_SysConfigTable.num_core, 1);	

	varValue total_power_adaptation_delta = c_inputs_table.measured_power[0] - estimated_reduced_total_power;
	varValue alpha_salita = 0.04f; //TODO: define and TUNE! these coefficients.
	varValue alpha_discesa = 0.08f;
	varValue total_power_adaptation_alpha = 0;

	if (total_power_adaptation_delta > 0)
	{
		total_power_adaptation_alpha = alpha_discesa;			
	}
	else
	{
		total_power_adaptation_alpha = alpha_salita;
	}
	//if (c_parameters_table.power_budget_changed)
	//{	
	//	total_power_adaptation_alpha += 0.2;
	//}
	if (c_parameters_table.pw_changed_alpha_counter > 0)
	{
		//TODO: formula of parabola thought 2 points and and line
			//for the moment linear //TODO GLOBAL ETC
		//TODO: also maybe interpolate the line: e.g. if the 2 pw budget are very different the 0.8 is 0.95, if they are 
		//		close it is better 0.5 or lower, etc.
		total_power_adaptation_alpha = (0.8f - total_power_adaptation_alpha) / 
		             PW_CHANGED_ALPHA_COUNTER * c_parameters_table.pw_changed_alpha_counter + total_power_adaptation_alpha;
 	}

	c_parameters_table.total_power_adaptation_term *= (VD_ONE - total_power_adaptation_alpha);
	c_parameters_table.total_power_adaptation_term += total_power_adaptation_delta * total_power_adaptation_alpha;	
}

varValue fComputeEstimatedTotalPower (varFor i_start, varFor i_end, varFor i_increment)
{
    varValue estimated_total_power = 0;

    for (varFor i = i_start; i < i_end; i+=i_increment)
	{
		//if (g_ControlConfigTable.variable_voltage_solution == VV_MOVING_AVERAGE)
        //{
            if (c_parameters_table.vv_moving_average_og_freq[i] == 0)
            {
                c_parameters_table.vv_moving_average_og_freq[i] = g_SysConfigTable.core_min_frequency;
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("Error, og freq core %d = 0\n\r", i);
#endif
            }

            //TODO: use formula below for int?
            //estimated_total_power += ( target_core_power[i] * (vv_moving_average_og_freq[i]*VD_100_PERC / computed_core_frequency[i]) ) / VD_100_PERC;
            varValue freq_reduction_coeff = c_parameters_table.computed_core_frequency[i] / c_parameters_table.vv_moving_average_og_freq[i];
            if (freq_reduction_coeff <= 0.01f) //TODO: check values
            {
 #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("Error < 0: ");
                printFloat(freq_reduction_coeff);
                printf(" - core: %d - ogf: ", i);
                printFloat(c_parameters_table.vv_moving_average_og_freq[i]);
                printf(" - af: ");
                printFloat(c_parameters_table.computed_core_frequency[i]);
                printf("\n\r");
#endif
                freq_reduction_coeff = 0.01f;
            }
            else if (freq_reduction_coeff > VD_ONE)
            {
 #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("Error > 0: ");
                printFloat(freq_reduction_coeff);
                printf(" - core: %d - ogf: ", i);
                printFloat(c_parameters_table.vv_moving_average_og_freq[i]);
                printf(" - af: ");
                printFloat(c_parameters_table.computed_core_frequency[i]);
                printf("\n\r");
#endif
                freq_reduction_coeff = VD_ONE;
            }

            estimated_total_power += c_parameters_table.target_core_power[i] * freq_reduction_coeff;
        //}
		//else
        //{
			//estimated_total_power += c_parameters_table.target_core_power[i];
		//}
	}

    return estimated_total_power;
}

varValue fCorePowerComputation(varFor i_start, varFor i_end, varFor i_increment){

	varValue total_power = 0;    

	//TBD: here we can also pass the pointer to the array and call the function only 1 time. The function will do for and sum.
	for (varFor i = i_start; i < i_end; i+=i_increment)
	{
        varValue power_formula_coeff[POWER_FORMULA_COEFF_NUM];

		#ifdef USE_INSTRUCTIONS_COMPOSITION
		for (varFor k = 0; k < POWER_FORMULA_COEFF_NUM; k++)
		{
			power_formula_coeff[k] = 0;

			for (varFor j = 0; j < g_SysConfigTable.core_wl_states_num; j++)
			{
				//power_formula_coeff[k] += i_ctrl_parameter_table->power_formula_coeff[ j ][k] * (varValue)i_ctrl_parameter_table->perc_workload[i][j] / VD_100_PERC;
				power_formula_coeff[k] += c_inputs_table.power_formula_coeff[ j ][k] * c_parameters_table.processed_workload[i][j] / VD_100_PERC;
			}

            //TODO
            c_inputs_table.core_ceff[i] = power_formula_coeff[1];
		}
		#else
		power_formula_coeff[0] = c_inputs_table.power_formula_coeff[0][0];
		power_formula_coeff[1] = c_inputs_table.core_ceff[i];
		#endif

		varValue core_power = lMathPowerCompute(c_inputs_table.target_freq[i], 
            c_parameters_table.computed_domain_voltage[g_SysConfigTable.core_domain_id[i]], 
            power_formula_coeff);

		if (core_power < c_parameters_table.target_core_power[i] - g_ControlConfigTable.max_pw_diff)
		{
			c_parameters_table.ctrl_cmd[i] = core_power / c_parameters_table.target_core_power[i];
		}
		else if (core_power > c_parameters_table.target_core_power[i] + g_ControlConfigTable.max_pw_diff)
		{
			c_parameters_table.ctrl_cmd[i] = core_power / c_parameters_table.target_core_power[i];
		}
		else
		{
			c_parameters_table.ctrl_cmd[i] = VD_ZERO;
		}

		//test_on_number
		if (core_power < g_SysConfigTable.core_idle_power)
		{
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Number issue, computed_core_power core %d: %f\n\r", i, core_power[i]);
			printf("Number issue, computed_core_power core %d: ", i);
			printFloat(core_power);
			printf("\n\r");
			#endif

			core_power = g_SysConfigTable.core_idle_power;
		} else if (core_power > g_SysConfigTable.core_max_power_single)
		{
			core_power = g_SysConfigTable.core_max_power_single;

			//Bitmap of Frequency Reduction.
    		c_telemetry_table.frequency_reduction_map[i] |= BM_FRM_MAX_SINGLE_POW_SAT;
		}
		
        c_parameters_table.target_core_power[i] = core_power;

        total_power += core_power;

		//TODO: for quadrants, we should change this to reflect quadrants disposition so we can add quadrant power also inside this for

		//Saturation check for single Core PowerMax Margin
	}

	return total_power;
}

void vVoltageCastVote(varFor i_start, varFor i_end, varFor i_increment, uint16_t* i_voltage_voting)
{
    for (varFor i = i_start; i < i_end; i+=i_increment)
	{
        varValue freq = c_inputs_table.target_freq[i];

		if (g_ControlConfigTable.variable_voltage_solution == VV_MOVING_AVERAGE)
        {
            freq -= c_parameters_table.vv_moving_average_freq_point[i];
        }
		
		//printf("freq: ");
		//printFloat(freq);
		//printf("\n\r");

		uint16_t domain = g_SysConfigTable.core_domain_id[i];
		varFor index = 0;
		while (index < g_SysConfigTable.num_voltage_levels)
		{
			if (freq <= g_SysConfigTable.voltage_table[index*3+2]) //TODO here just lloking at the min freq, not considering voltage overheads or other changes
			{
				i_voltage_voting[index + domain*g_SysConfigTable.num_voltage_levels]++;
				break; //index = g_SysConfigTable.num_voltage_levels+1;
			}
			else
				index++;
		}
		//checks
		if (index == g_SysConfigTable.num_voltage_levels)
		{
			//TODO error > max!

			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Error voltage choice core %d > max\n\r", i);
			#endif

			//TODO: put target at max

			//casting vote:
			i_voltage_voting[index-1 + domain*g_SysConfigTable.core_domain_id[i]]++;
		}
	}

}

void vVoltageChoice(varFor i_start, varFor i_end, varFor i_increment, uint16_t* i_voltage_voting)
{
    //TODO here I assumed performance level is already the percentage
	//TODO: voltage decision is not done correctly!! It may want to select a Voltage that is not the one of ANY core, 
	//		but it is still in an intermediate point, e.g. 8 cores at 0.5V, 1core at 1.2V, now it is either 0.5 or 1.2, but
	//		I may like to do 0.75V for example
	// instead of doing votes/N -> perc/100; I convert in votex*100 -> perc*N, so I have no div and no issue with int numbers.

    //  uint32_t performance_perc = c_inputs_table.perfmormance_level;

    for (varFor i = i_start; i < i_end; i+=i_increment)
	{
		uint32_t accumulator = 0;
		//uint16_t domain = i_ctrl_parameter_table->domain[i];

		varFor index = g_SysConfigTable.num_voltage_levels-1;
		while (index >= 0)
		{
			accumulator += (uint32_t)i_voltage_voting[index + i*g_SysConfigTable.num_voltage_levels] * 100;
			//printf("Accumulator: %d\n\r", accumulator);
			if (accumulator >= c_inputs_table.perfmormance_level * g_SysConfigTable.num_cores_per_domain[i])
			{
				c_parameters_table.computed_domain_voltage[i] = g_SysConfigTable.voltage_table[index*3+0]; //TODO here just lloking at the min freq, not considering voltage overheads or other changes
                c_parameters_table.store_max_freq[i] = g_SysConfigTable.voltage_table[index*3+2];
				break; //index = -1;
			}
			else
				index--;
		}
		//checks
		if (index < 0)
		{
			//TODO error 

			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Error voltage choice domain %d < min\n\r", i);
			#endif

			//TODO: put target at min

			c_parameters_table.computed_domain_voltage[i] = g_SysConfigTable.voltage_table[0*3+0]; //TODO here just lloking at the min freq, not considering voltage overheads or other changes
            c_parameters_table.store_max_freq[i] = g_SysConfigTable.voltage_table[0*3+2];
		}
	}

    #ifdef FIX_VOLTAGE
	for (varFor i = 0; i < g_SysConfigTable.num_pw_domains; i++)
	{
		c_parameters_table.computed_domain_voltage[i] = g_SysConfigTable.voltage_table[(6-1)*3+0];
		c_parameters_table.store_max_freq[i] = g_SysConfigTable.voltage_table[(6-1)*3+2];
	}
	#endif //FIX_VOLTAGE
}

void vDummyPowerReduction(varFor i_start, varFor i_end, varFor i_increment, varValue delta, varValue tot_power)
{
	varValue perc = delta / tot_power;

	for (varFor i = i_start; i < i_end; i+=i_increment)
	{
		varValue reduction = c_parameters_table.target_core_power[i] * perc;
		c_parameters_table.target_core_power[i] -= reduction;

		//TODO: Canc:
		#ifdef ALPHA_DEBUG
		varValue* mem_address = NULL;
		mem_address = (varValue*)IMP_ADR_OUT_FIRST_ALPHA;
		size_t address_num_of_bytes = sizeof(varValue);
	    //TODO: Compiler doesn't work properly: it multiply everything for 4.
	    address_num_of_bytes = 1;

	    mem_address += i*address_num_of_bytes;
	    if (mem_address > (varValue*)IMP_ADR_OUT_ALPHA_LAST)
        {
	    	printf("Error writing alpha\r\n");
	    }

	    *mem_address = perc;

	    mem_address = (varValue*)IMP_ADR_OUT_FIRST_REDPW;

	    mem_address += i*address_num_of_bytes;
	    if (mem_address > (varValue*)IMP_ADR_OUT_REDPW_LAST)
        {
	    	printf("Error writing alpha\r\n");
	    }

	    *mem_address = c_parameters_table.target_core_power[i];        
		#endif //ALPHA_DEBUG
	}
}

void vAlphaPowerReduction(varValue i_delta_power){
	
    varValue accumulation_Alpha = 0;
    varValue Alpha[MAX_NUM_CORE];

	//Compute Alpha
    //TODO split for cluster need fix
    accumulation_Alpha = fComputeAlpha(0, g_SysConfigTable.num_core, 1, i_delta_power, Alpha);

    if (g_ControlConfigTable.use_freq_binding == PCF_TRUE)
    {
        //TODO: THIS IS WRONG, NEED A DIFFERENT ALGORITHM
        //vComputeAlphaBindings();
    }

	//Normalize Alpha
	vNormalizeAlpha(0, g_SysConfigTable.num_core, 1, Alpha, accumulation_Alpha);

	//Apply Alpha Reduction
	vApplyAlphaReduction(0, g_SysConfigTable.num_core, 1, Alpha, i_delta_power);

	//return TotalPower;
}

varValue fComputeAlpha(varFor i_start, varFor i_end, varFor i_increment, varValue delta_power, varValue* Alpha)
{
    varValue accumulation_Alpha = 0;

	for (varFor i = i_start; i < i_end; i+=i_increment)
	{
		//test_on_number //MUST do always, not only when test on numbers, and it is not a ToN Error!
		if ((g_SysConfigTable.core_critical_temperature - g_ControlConfigTable.pid_temperature_margin - c_parameters_table.measured_temperature[i]) 
				> 0)
        {
            Alpha[i] = 1 / (g_SysConfigTable.core_critical_temperature - c_parameters_table.measured_temperature[i]);
        }
        else
		{
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Number issue, i_measured_temperature > l_core_critical_temperature, core %d: %f\n\r", i, i_measured_temperature[i]);
			//printf("Number issue, i_measured_temperature > l_core_critical_temperature, core %d: ", i);
			//printFloat(c_parameters_table.measured_temperature[i]);
			//printf("\n\r");
			#endif

			Alpha[i] = 1 / (g_ControlConfigTable.pid_temperature_margin);
		}

		//Test on Alpha acceptability:
		if (delta_power*Alpha[i] > c_parameters_table.target_core_power[i] - g_SysConfigTable.core_idle_power)
		{
			Alpha[i] = (c_parameters_table.target_core_power[i] - g_SysConfigTable.core_idle_power) / delta_power;

			//TODO add "not reduction" in a bin, so I know I have to reduce more!
		}
        
		//test_on_number
		if (Alpha[i] <= 0)
		{
			/* TODO:
			//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
			ErrorMap |= BM_ERROR_NUMBERS_VALUE;
			#endif
			*/

			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Number issue, Alpha<0, core %d: %f\n\r", i, Alpha[i]);
			printf("Number issue, Alpha<0, core %d: ", i);
			printFloat(Alpha[i]);
			printf("\n\r");
			#endif

			Alpha[i] = g_ControlConfigTable.alpha_error_base_value; //TBD: we are whitin an ERROR!!! do not put 0.01!

		} else if (Alpha[i] > g_ControlConfigTable.alpha_max_value)
		{
			/* TODO:
			//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
			ErrorMap |= BM_ERROR_NUMBERS_VALUE;
			#endif
			*/
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Number issue, Alpha>1, core %d: %f\n\r", i, Alpha[i]);
			printf("Number issue, Alpha>1, core %d: ", i);
			printFloat(Alpha[i]);
			printf("\n\r");
			#endif

			Alpha[i] = g_ControlConfigTable.alpha_max_value;
		}

		accumulation_Alpha += Alpha[i];
		//TODO: for quadrants, we should change this to reflect quadrants disposition so we can add quadrant power also inside this for
	}

    return accumulation_Alpha;
}

void vComputeAlphaBindings(void)
{
    //TBU: This uint16+ is VERY delicate, it has to change with the
    //	typedef struct _ctrl_parameter_table, so it could need a better design

    // varCoreBinding* i_core_binding_vector = i_ctrl_parameter_table->core_binding_vector;
    // varValue min_group_alpha[(MAX_NUM_CORE / 2) + 1]; // +1 cuz of 0

    // varValue accumulation_Alpha = 0;

    //TODO: Optimize This
    //Initialization
    // for (varFor i = 0; i < (g_SysConfigTable.num_core / 2) +1; i++ )
    //     min_group_alpha[i] = VD_ONE;

    // // Scan
    // for (varFor i = 0; i < g_SysConfigTable.num_core; i++)
    // {
    //     if (i_core_binding_vector[i] != 0)
    //     {
    //         //test_on_number
    //         if ((i_core_binding_vector[i] > (g_SysConfigTable.num_core/2)) /*|| (i_core_binding_vector[i] < 0) since it is unsigned, is never <0*/)
    //         {
    //             //TODO
    //             //ErrorMap |= BM_ERROR_NUMBERS_VALUE;
    //             #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //             printf("Number issue, i_core_binding_vector>g_SysConfigTable.num_core/2, core %d: %d\n\r", i, i_core_binding_vector[i]);
    //             #endif

    //             i_core_binding_vector[i] = 0;
    //         }

    //         if ((Alpha[i] < min_group_alpha[i_core_binding_vector[i]]))
    //         {
    //             min_group_alpha[i_core_binding_vector[i]] = Alpha[i];
    //         }
    //     }
    // }	//for

    // // Apply
    // for (varFor i = 0; i < g_SysConfigTable.num_core; i++)
    // {
    //     if (i_core_binding_vector[i] != 0)
    //     {
    //         Alpha[i] = min_group_alpha[i_core_binding_vector[i]];

    //         //todo
    //         i_telemetry_struct->frequency_reduction_map[i] |= BM_FRM_BINDING_RED;
    //     }

    //     accumulation_Alpha += Alpha[i];
    // }
}

void vNormalizeAlpha(varFor i_start, varFor i_end, varFor i_increment, varValue* Alpha, varValue i_accumulation_Alpha)
{
    for (varFor i = i_start; i < i_end; i+=i_increment)
	{
		// Alpha Normalized. For optimization we use Alpha[i]
		Alpha[i] = Alpha[i] / i_accumulation_Alpha;
		//test_on_number
		if (Alpha[i] <= 0)
		{
			/* TODO:
			//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
			ErrorMap |= BM_ERROR_NUMBERS_VALUE;
			#endif
			*/
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Number issue, Alpha<0, core %d: %f\n\r", i, Alpha[i]);
			printf("Number issue, Alpha<0, core %d: ", i);
			printFloat(Alpha[i]);
			printf("\n\r");
			#endif

			Alpha[i] = g_ControlConfigTable.alpha_error_base_value; //TBD: we are whitin an ERROR!!! do not put 0.01!

		} else if (Alpha[i] >= 1)
		{
		    	/* TODO:
			//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
			ErrorMap |= BM_ERROR_NUMBERS_VALUE;
			#endif
			*/
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Number issue, Alpha>1, core %d: %f\n\r", i, Alpha[i]);
			printf("Number issue, Alpha>1, core %d: ", i);
			printFloat(Alpha[i]);
			printf("\n\r");
			#endif

			Alpha[i] = g_ControlConfigTable.alpha_error_base_value; //TBD: we are whitin an ERROR!!! do not put 0.99!
		}

	}
}

void vApplyAlphaReduction(varFor i_start, varFor i_end, varFor i_increment, varValue* Alpha, varValue i_delta_power)
{
    for (varFor i = i_start; i < i_end; i+=i_increment)
	{
		// Updated target_core_power
		c_parameters_table.target_core_power[i] -= (Alpha[i] * i_delta_power);
		//test_on_number
		if (c_parameters_table.target_core_power[i] <= 0)
		{
			/* TODO:
			//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
			ErrorMap |= BM_ERROR_NUMBERS_VALUE;
			#endif
			*/
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Number issue, o_reduced_core_power<0, core %d: %f\n\r", i, o_reduced_core_power[i]);
			printf("Number issue, reduced target_core_power<0, core %d: ", i);
			printFloat(c_parameters_table.target_core_power[i]);
			printf(" target: ");
			printFloat(c_parameters_table.target_core_power[i]+(Alpha[i] * i_delta_power));
			printf(" alpha: ");
			printFloat(Alpha[i]);
			printf(" delta: ");
			printFloat(i_delta_power);
			printf("\n\r");
			#endif

			c_parameters_table.target_core_power[i] = g_SysConfigTable.core_idle_power;
		}

		//todo
		c_telemetry_table.frequency_reduction_map[i] |= BM_FRM_ALPHA_RED;

		//TODO: if no bindings (def) we should compute here the new estimated_total_power += target_core_power[i];, otherwise do after bindings

		//TODO: Canc:
		#ifdef ALPHA_DEBUG
		varValue* mem_address = NULL;
		mem_address = (varValue*)IMP_ADR_OUT_FIRST_ALPHA;
		size_t address_num_of_bytes = sizeof(varValue);
	    //TODO: Compiler doesn't work properly: it multiply everything for 4.
	    address_num_of_bytes = 1;

	    mem_address += i*address_num_of_bytes;
	    if (mem_address > (varValue*)IMP_ADR_OUT_ALPHA_LAST)
        {
	    	printf("Error writing alpha\r\n");
	    }

	    *mem_address = Alpha[i];

	    mem_address = (varValue*)IMP_ADR_OUT_FIRST_REDPW;

	    mem_address += i*address_num_of_bytes;
	    if (mem_address > (varValue*)IMP_ADR_OUT_REDPW_LAST)
        {
	    	printf("Error writing alpha\r\n");
	    }

	    *mem_address = c_parameters_table.target_core_power[i];        
		#endif //ALPHA_DEBUG
	}
}

void vComputeTemperatureReduction(varFor i_start, varFor i_end, varFor i_increment)
{
	for (varFor i = i_start; i < i_end; i+=i_increment)
	{
		// c_parameters_table.target_core_power[i]; //TODO: Optimize this....
        varValue core_reduced_value;

		//PID
		core_reduced_value = lMathPidCompute(c_parameters_table.target_core_power[i], 
 		           c_parameters_table.measured_temperature[i], i, c_parameters_table.ctrl_cmd[i], c_parameters_table.power_budget_changed);

		//TestOnNumber
		if ( (core_reduced_value <= 0) || (core_reduced_value > c_parameters_table.target_core_power[i]) )
		{
			c_error_map |= BM_ERROR_NUMBERS_VALUE;
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Number issue, target_core_power, core %d: %f\n\r", i, target_core_power[i]);
			printf("Number issue, target_core_power, core %d: ", i);
			printFloat(c_parameters_table.target_core_power[i]);
			printf("\n\r");
			#endif

			c_parameters_table.target_core_power[i] = g_SysConfigTable.core_idle_power; //TBD: maybe too much conservative in case > ?? (consider still there is an error!)
		}

		if (core_reduced_value < c_parameters_table.target_core_power[i]) //TODO
		{
			c_telemetry_table.frequency_reduction_map[i] |= BM_FRM_PID_TEMP_RED;
		}

        c_parameters_table.target_core_power[i] = core_reduced_value;
	}
}

void vComputeFrequency(void)
{
	//#ifdef USE_COUPLING_SOLUTION
    if (g_ControlConfigTable.variable_voltage_solution == VV_COUPLING_SOLUTION)
    {
        vFrequencyCouplingTransformation(0, g_SysConfigTable.num_pw_domains, 1, 16, 0.05, 0.05);
    }
    else if (g_ControlConfigTable.variable_voltage_solution == VV_MOVING_AVERAGE)
    {
        vFrequency2PowerConversion(0, g_SysConfigTable.num_core, 1);
    }
	else
	{
		//TODO

		printf("ERRORRRRRR \n\r"); //TODO
	}

    /*** Check Bindings between Cores ***/
	//TODO
	if (g_ControlConfigTable.use_freq_binding == PCF_TRUE) //TODO create a local variable. Also check the code.
	{
        //TODO
		//vCoreBindingReduction(computed_core_frequency, &c_parameters_table, &c_telemetry_table);
	}

    //#ifndef USE_COUPLING_SOLUTION
	//update freq_diff_red paramenter:
    //TODO: optimize for parallel
	varValue ma_alpha = g_ControlConfigTable.vv_moving_average_alpha;
	//WHOLE, for power budget change
	if (c_parameters_table.pw_changed_alpha_counter > 0)
	{
		//TODO: formula of parabola thought 2 points and and line
		//for the moment linear //TODO GLOBAL ETC
		//TODO: also maybe interpolate the line: e.g. if the 2 pw budget are very different the 0.8 is 0.95, if they are 
		//         close it is better 0.5 or lower, etc.
		ma_alpha = (0.9f - g_ControlConfigTable.vv_moving_average_alpha) / 
                (PW_CHANGED_ALPHA_COUNTER) * c_parameters_table.pw_changed_alpha_counter 
				+ g_ControlConfigTable.vv_moving_average_alpha;

		c_parameters_table.pw_changed_alpha_counter--; //After
	}
	for (varFor i = 0; i < g_SysConfigTable.num_core; i++)
	{
		varValue ma_alpha_c = ma_alpha;
		//for single TF change
		if (c_inputs_table.target_freq_changed[i] == PCF_TRUE)
		{
			//if (g_ControlConfigTable.variable_voltage_solution == VV_MOVING_AVERAGE)
			//if (c_parameters_table.prev_power_budget  < c_inputs_table.total_power_budget) //TBC: only <, because this fix working good on increasing the power budget, but bad on decreasing it.
			//	{
				
				c_parameters_table.tf_changed_alpha_counter = TF_CHANGED_ALPHA_COUNTER; //16
		}
		if (c_parameters_table.tf_changed_alpha_counter > 0)
		{
			//TODO: formula of parabola thought 2 points and and line
			//for the moment linear //TODO GLOBAL ETC
			//TODO: also maybe interpolate the line: e.g. if the 2 pw budget are very different the 0.8 is 0.95, if they are 
			//         close it is better 0.5 or lower, etc.
			ma_alpha_c = (1.0f - ma_alpha_c) / 
					(TF_CHANGED_ALPHA_COUNTER) * c_parameters_table.tf_changed_alpha_counter 
					+ ma_alpha_c;
			
			c_parameters_table.tf_changed_alpha_counter--; //After
		}
		//Moving average to avoid oscillation
		//if instant instead, just use alpha = 1
		c_parameters_table.vv_moving_average_freq_point[i] *= (VD_ONE - ma_alpha_c);
		//TODO: "with" Turbo Boost
		//c_parameters_table.vv_moving_average_freq_point[i] += 
        //    (c_inputs_table.target_freq[i] - c_parameters_table.computed_core_frequency[i]) * ma_alpha_c;
		//without:
		c_parameters_table.vv_moving_average_freq_point[i] += ((c_inputs_table.target_freq[i] - c_parameters_table.computed_core_frequency[i])>0)?
            ((c_inputs_table.target_freq[i] - c_parameters_table.computed_core_frequency[i]) * ma_alpha_c):0;

	}
}

void vFrequencyCouplingTransformation(varFor i_start, varFor i_end, varFor i_increment,
                                    int i_max_iterations, varValue i_tolx, varValue i_tolf )
{
	//EMPTY
}


varFor lPopulateCouplingStructure(varFor i_start, varFor i_end, varFor i_increment, varFor i_domain, struct coupling_p2f *i_func_param)
{
    
	//EMPTY

}

void vProcessFrequency(varFor i_core, varValue i_frequency)
{
    //Naif Way to smooth the Frequency increase
	//TODO: improve this

    if (i_frequency > c_parameters_table.computed_core_frequency[i_core] + g_ControlConfigTable.freq_max_gradient)
    {
        c_parameters_table.computed_core_frequency[i_core] += g_ControlConfigTable.freq_max_gradient;
        c_telemetry_table.frequency_reduction_map[i_core] |= BM_FRM_MAX_GRADIENT;
    }
    else
    {
        c_parameters_table.computed_core_frequency[i_core] = i_frequency;
    }

    //TODO: can I optimize this formula? Because I have to pass all the time the address of the Formula Coeff which are fixed per 2 iteration of the control task

    //Hysteresis Check for the temperature going higher than threshold
    //ATTENTION!!! Not thought values can make the PID not to work!
    if ( (c_parameters_table.measured_temperature[i_core] >= g_ControlConfigTable.hyst_high_temp_limit) || 
            (c_parameters_table.hyst_thresh_reached[i_core]) )
    {
        if (c_parameters_table.measured_temperature[i_core] < g_ControlConfigTable.hyst_low_temp_limit)
        {
            c_parameters_table.hyst_thresh_reached[i_core] = PCF_FALSE;
        }
        else
        {
            c_parameters_table.hyst_thresh_reached[i_core] = PCF_TRUE;
            c_parameters_table.computed_core_frequency[i_core] = g_SysConfigTable.core_min_frequency; //TBD:

            c_telemetry_table.frequency_reduction_map[i_core] |= BM_FRM_HYST_EMERG;
        }
    }

    if (c_parameters_table.computed_core_frequency[i_core] < g_SysConfigTable.core_min_frequency)
    {
        //TBC: is this really an error value?
        c_error_map |= BM_ERROR_NUMBERS_VALUE;
        //printf("Number issue, computed_core_frequency, core %d: %d\n\r", i, (int)(computed_core_frequency[i_core]*1000.0f));
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
        //printf("Number issue, computed_core_frequency, core %d: %f\n\r", i, computed_core_frequency[i_core]);
        printf("Number issue, computed_core_frequency, core %d: ", i_core);
        printFloat(c_parameters_table.computed_core_frequency[i_core]);
        printf("\n\r");
        #endif

        c_parameters_table.computed_core_frequency[i_core] = g_SysConfigTable.core_min_frequency;

    }else if (c_parameters_table.computed_core_frequency[i_core] > g_SysConfigTable.core_max_frequency)
    {
        //TBC: is this really an error value?
        c_error_map |= BM_ERROR_NUMBERS_VALUE;
        //printf("Number issue, computed_core_frequency, core %d: %d\n\r", i, (int)(computed_core_frequency[i_core]*1000.0f));
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
        //printf("Number issue, computed_core_frequency, core %d: %f\n\r", i, computed_core_frequency[i_core]);
        printf("Number issue, computed_core_frequency, core %d: ", i_core);
        printFloat(c_parameters_table.computed_core_frequency[i_core]);
        printf("\n\r");
        #endif

        c_parameters_table.computed_core_frequency[i_core] = g_SysConfigTable.core_max_frequency;

        //for the power filtering
        //pw_filter_removed_freq[i_core] += (target_single_frequency*VD_100_PERC) / (computed_core_frequency[i_core]*VD_100_PERC);
    }

}

void vFrequency2PowerConversion(varFor i_start, varFor i_end, varFor i_increment)
{
    for (varFor i = i_start; i < i_end; i+=i_increment)
	{
        varValue power_formula_coeff[POWER_FORMULA_COEFF_NUM];
        //TODO: improve this
        power_formula_coeff[0] = c_inputs_table.power_formula_coeff[0][0];
        power_formula_coeff[1] = c_inputs_table.core_ceff[i];
		
		varValue target_single_frequency = lMathFrequencyCompute(c_parameters_table.target_core_power[i], 
                c_parameters_table.computed_domain_voltage[g_SysConfigTable.core_domain_id[i]],
                power_formula_coeff);

		c_parameters_table.vv_moving_average_og_freq[i] = target_single_frequency;

		/*
		//TBD MOVED HERE FROM BELOW
		// not-proper-analisys: it is better below
		//CUGV //TODO move to a better place.
		varValue alpha = g_ControlConfigTable.freq_diff_alpha;

		//update freq_diff_red paramenter:
		//Moving average to avoid oscillation
		//if instant instead, just use alpha = 1
		c_parameters_table.freq_diff_red[i] *= (VD_ONE - alpha);
		c_parameters_table.freq_diff_red[i] += (c_parameters_table.target_freq[i] - target_single_frequency) * alpha;
		*/

		/*
		if (i==0){
		printf("Fc: ");
		printFloat(target_single_frequency);
		printf("\n\r");}
		*/

		vProcessFrequency(i, target_single_frequency);
	}
}

void vTelemetryManagement(void)
{
	/*** B.2: Write Accumulation Values ***/
	//Check if the Window Horizon for the measurement is reached
	//TBD: Better to do 2 for(36) and 1 if, or 1 for and 36if? If the second is better, move this if inside the for above.
	if (c_parameters_table.telemetry_counter <= 0) //TBD: <= or == ? <= also take into account some errors.
	{
		c_parameters_table.telemetry_counter = c_inputs_table.telemetry_horizon;

		c_telemetry_table.chip_avg_estimated_power = 0;
        //TODO: Optimize this for parallel
		for (varFor i = 0; i < g_SysConfigTable.num_core; i++)
		{
			//TBD: it is better to have a new variable for (varValue)TelemetryHorizion or it is ok like this?
			c_telemetry_table.core_avg_sensor_data[i].frequency 	/= (varValue)c_inputs_table.telemetry_horizon;
			c_telemetry_table.core_avg_sensor_data[i].voltage 	    /= (varValue)c_inputs_table.telemetry_horizon;
			c_telemetry_table.core_avg_sensor_data[i].temperature  /= (varValue)c_inputs_table.telemetry_horizon;

			c_telemetry_table.chip_avg_estimated_power			    += c_telemetry_table.core_avg_estimated_power[i];
			c_telemetry_table.core_avg_estimated_power[i] 		    /= (varValue)c_inputs_table.telemetry_horizon;
		}
		c_telemetry_table.chip_avg_estimated_power 				/= (varValue)c_inputs_table.telemetry_horizon;

		//test_on_number
        //TODO
        /*
		if ((c_telemetry_table.chip_avg_estimated_power) > (g_SysConfigTable.core_max_power_single * (varValue)g_SysConfigTable.num_core) ) 
		{
			l_error_map |= BM_ERROR_NUMBERS_VALUE;
			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			//printf("Number issue, chip_avg_estimated_power: %f\n\r", c_telemetry_table.chip_avg_estimated_power);
			printf("Number issue, chip_avg_estimated_power: ");
			printFloat(c_telemetry_table.chip_avg_estimated_power);
			printf("\n\r");
			#endif

			//TODO: something
		}
        */

		//TODO: DATE_PAPER, remove this.
		bImpWriteFreqRedMap(&c_telemetry_table);

		if (bWriteGlobalVariable(&c_telemetry_table, G_TELEMETRY, GLOBAL_WRITE_ALL, sizeof(c_telemetry_table), c_task_id) != PCF_TRUE)
		{
			//TODO

			#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
			printf("bWriteGlobalVariable issue: c_telemetry_table, id: %d\n\r", c_task_id);
			#endif
		}

		// Resetting the Accumulation Internal Variables
        //TODO: optimize for cluster
		for (varFor i = 0; i < g_SysConfigTable.num_core; i++)
		{
			c_telemetry_table.core_avg_sensor_data[i].frequency 	= 0;
			c_telemetry_table.core_avg_sensor_data[i].voltage 		= 0;
			c_telemetry_table.core_avg_sensor_data[i].temperature 	= 0;

			c_telemetry_table.core_avg_estimated_power[i] 			= 0;

			//TBD: the FRM tells the SUM of the story or just the last iteration??
			c_telemetry_table.frequency_reduction_map[i]			= BM_RESET;
		}
		c_telemetry_table.chip_avg_estimated_power					= 0;
	}

	/*** Not sure the right position for this code, it may be put while waiting for the DMA ***/
	//TBD: we don't need to do it every cycle? only when bmc ask? Do I or everyhing?

	//write telemetry
}


