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

#ifndef _PCF_CTRL_CONFIG_H_
#define _PCF_CTRL_CONFIG_H_


/* Libraries Inclusion */
#include "cfg_types.h"
#include "cfg_system.h"


/** Definitions **/
#define LMS_CN									(PCF_WL_STATES*PCF_MODEL_COEFF)
#define LMS_CNxCN								(LMS_CN*LMS_CN)


/******************************/
/*** Control Configuration ****/
/******************************/
struct ctrl_config_param {

	/***** CTRL Configuration *****/
	varBool_e use_lms;
	varBool_e use_freq_binding;					// to use Frequency Bindings between Cores
    varBool_e use_quadrant_pw_budget;

	/***** PID Configuration ******/
	varBool_e use_pid_integral_action;					// to use Integral Action inside PID
	varBool_e use_pid_derivative_action;				// to use Derivative Action inside PID
	varBool_e use_pid_saturation;						// to use Output Saturation inside PID
	//TBC: Since our implementation of the PID, maybe is better to not make this definable, but to FORCE it active.
	varBool_e use_pid_ff_action;						// to use FeedForward Action inside PID
	varBool_e use_pid_anti_windup_sat;					// to use Anti Wind-up of the Integrator, Saturation kind

	uint8_t variable_voltage_solution;

	/*** Telemetry Configuration **/
	varBool_e use_frequency_reduction_map;				// to collect the information on the Frequency Reduction causes

};

struct ctrl_thermal_param {

	/* Initialization Values */
	// They have to be defined even if PID_CONFIG_x is disabled //TODO: fix this?
	varValue pid_kp;
	varValue pid_ki;
	varValue pid_kd;
	varValue pid_dt;
	varValue pid_anti_windup_sat_coeff;
	varValue pid_anti_windup_sat_up;

	varValue pid_integral_reduction_coeff;

	/*** Other Control Values *****/
	varValue pid_temp_margin;			// A margin to the above Temperature

	varValue hyst_high_temp_limit;
	varValue hyst_low_temp_limit;

	varFor fuzzy_mul;
	varFor der_mul;

	varValue fuzzy_temp_lim[4];
	varValue fuzzy_der_lim[4];
	varFor fuzzy_value_table[25]; //(4+1)*(4+1) //TODO parametrize?

};

struct ctrl_power_param {

	/***** Filters and Moving Average ******/
	varValue moving_average_alpha;
	
	varValue alpha_max_value;
	varValue alpha_error_base_value;

	varValue max_pw_diff;
};

struct ctrl_input_param {

	
};

struct ctrl_output_param {

	varValue freq_max_gradient;
};

struct ctrl_ident_param {
	/** Configuration **/
	//uint16_t coeff_num;
	uint16_t batch_iteration_num;

	/* Values */
	// Forgetting Factor
	varValue lambda;
	varValue bound_limits[PCF_WL_STATES][PCF_MODEL_COEFF];
};

typedef struct _ctrl_config_table {

	struct ctrl_config_param	conf;

	struct ctrl_thermal_param 	therm;
	struct ctrl_power_param 	pw;

	struct ctrl_input_param 	input;
	struct ctrl_output_param 	output;

	struct ctrl_ident_param		ident;

} ctrl_config_table_t;


/******************************/
/***** Control Structures *****/
/******************************/

struct ctrl_commands {

	varValue 		target_freq[PCF_CORES_MAX];
	varBool_e 		target_freq_changed[PCF_CORES_MAX];

	varValue 		total_power_budget;
    varValue 		quadrant_power_budget[PCF_DOMAINS_MAX];

    varCoreBinding 	core_binding_vector[PCF_CORES_MAX];
	
    uint8_t 		perfmormance_level;

	varFor 			telemetry_horizon;
};

struct temperature_measures {

	varValue core[PCF_CORES_MAX];
};

struct power_measures {
	varValue total;
	varValue domain[PCF_DOMAINS_MAX];
};

struct performance_measures {

	#ifdef USE_INSTRUCTIONS_COMPOSITION
	uint8_t 	perc[PCF_CORES_MAX][PCF_WL_STATES];
	#endif
	varValue 	ceff[PCF_CORES_MAX];
};

/* Main one */
struct ctrl_measures {

	//TBD: pointers or variables?
	struct temperature_measures 	temp;
	struct power_measures 			pw;
	struct performance_measures		perf;	
};

typedef struct _ctrl_inputs {
	struct ctrl_commands cmd;
	struct ctrl_measures msr;
} ctrl_inputs_t;

struct ctrl_input_process {

	//Power Adaptation
	varValue prev_power_budget;
	varBool_e power_budget_changed;
	varValue total_power_adaptation_term;
	varValue prev_tot_power[2];
	varValue workload[PCF_CORES_MAX];
	varValue temp_der[PCF_CORES_MAX];
	varValue prev_temp[PCF_CORES_MAX];
};

struct ctrl_moving_average {

	//Moving Average
	varValue alpha;
	varFor alpha_counter;
	varValue og_freq[PCF_CORES_MAX];
	varValue freq_point[PCF_CORES_MAX];
};

struct ctrl_thermal {

	//Control
	varValue ctrl_cmd[PCF_CORES_MAX];
	varBool_e hyst_thresh_reached[PCF_CORES_MAX];

	//PID
	varValue pid_integral_error[PCF_CORES_MAX];
	varValue ki;
	varValue pid_previous_error[PCF_CORES_MAX];
	varValue kd;
	varValue saturation_max;
	varValue Pidle;

	varValue kp;
	varValue Tcrit_pid;

	varFor fuzzy_vcred[PCF_CORES_MAX]; 
};

struct ctrl_output {

	//Outputs:
	varValue computed_domain_voltage[PCF_DOMAINS_MAX];
	varValue computed_core_frequency[PCF_CORES_MAX];

	//TBD:
	varValue store_max_freq[PCF_DOMAINS_MAX];
	varFor store_voltage_level[PCF_DOMAINS_MAX];
};

typedef struct _ctrl_values_table {

	varValue target_core_power[PCF_CORES_MAX];

	struct ctrl_input_process 	ip;
	struct ctrl_thermal			th;
	struct ctrl_moving_average 	ma;

	struct ctrl_output 			op;

	varFor telemetry_counter;
	varFor ctrl_counter;
	varValue core_frequency[PCF_CORES_MAX];
	varFreqRedMap freq_red_map[PCF_CORES_MAX];

} ctrl_values_table_t;


/******************************/
/********* Telemetry **********/
/******************************/

/*******************/
/*** Data Struct ***/
/*******************/
struct ctrl_data {
	varValue freq;
	varValue voltage;
	varValue temp;

	uint16_t id;
	varValue est_pw;
};

typedef struct _telemetry {
	//TODO: Need to adjust this because we are passing only 1 Power (either Total or Mean)
	struct ctrl_data avg_core[PCF_CORES_MAX];
	struct ctrl_data avg_domain[PCF_DOMAINS_MAX];
	varValue chip_est_pw;

	varFreqRedMap frequency_reduction_map[PCF_CORES_MAX];
	uint8_t core_perf_grade[PCF_CORES_MAX];

    //TODO: add all the other values.
	//varValue chip_avg_measured_power;
} telemetry_t;

/******************************/
/***** Global Variables *******/
/******************************/
extern ctrl_config_table_t g_ControlConfigTable;
extern ctrl_values_table_t g_TasksCtrlInput;
extern ctrl_inputs_t g_input_ctrl;

/******************************/
/******* API Functions ********/
/******************************/

//varBool_e bCtrlResetStruct(void *addr, pcf_ctrl_struct type);
//varBool_e bCtrlInitStruct(void *addr, pcf_ctrl_struct type);
//varBool_e bCtrlResetTelem(void *addr, pcf_ctrl_struct type);
//varBool_e bCtrlInitTelem(void *addr, pcf_ctrl_struct type);

//TODO: move
//void vSecInitCtrlParam(void *addr, pcf_ctrl_struct type);




#endif //lib #ifdef
