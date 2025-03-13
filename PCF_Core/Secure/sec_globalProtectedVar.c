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

//FULL TODO: just to see if I fix this way the definition problem.

/* FreeRTOS Inclusions. */
#include <FreeRTOS.h>
#include <semphr.h>


#include "sec_globalProtectedVar.h"
/* Libraries Inclusion */
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_system.h"
#include "cfg_control.h"


config_sys_t g_config_sys =
{
    .num_cores                              = PCF_CORES_MAX,
    .num_domains                            = PCF_DOMAINS_MAX,
    .num_chiplets                         	= 1,
    .num_sockets                        	= 1,
    .num_voltage_levels                     = 15,

    #if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
    .voltage_table                          =
        {0.50f, 0.40f, 1.35f, 
         0.55f, 0.40f, 1.60f, 
         0.60f, 1.35f, 1.80f,
         0.65f, 1.35f, 2.00f,
         0.70f, 1.35f, 2.20f,
         0.75f, 1.60f, 2.40f,
         0.80f, 1.80f, 2.60f,
         0.85f, 1.80f, 2.75f,
         0.90f, 2.00f, 2.90f,
         0.95f, 2.00f, 3.05f,
         1.00f, 2.00f, 3.20f,
         1.05f, 2.60f, 3.35f,
         1.10f, 2.60f, 3.45f,
         1.15f, 2.60f, 3.55f,
         1.20f, 2.60f, 3.66f
     },

    .num_cores_per_domain                   = {8,8,8,8},
    .core_domain_id                         = 
    /* 1 DOMAIN */
    /*
                                        { 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0 },
    */
    /* 4 DOMAINS */
                                        { 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        1, 1, 1, 1, 1, 1, 1, 1, 1,
                                        2, 2, 2, 2, 2, 2, 2, 2, 2,
                                        3, 3, 3, 3, 3, 3, 3, 3, 3 },
    
    
    /* ALL DOMAINS */
    /*
                                        { 0, 1, 2, 3, 4, 5, 6, 7, 8,
                                        9, 10, 11, 12, 13, 14, 15, 16, 17,
                                        18, 19, 20, 21, 22, 23, 24, 25, 26,
                                        27, 28, 29, 30, 31, 32, 33, 34, 35 },
    */
    /* OG RHEA */
    /*
                                        { 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        1, 1, 1, 1, 1, 1, 1, 1, 1,
                                        1, 1, 1, 1, 1, 1, 1, 1, 1,
                                        1, 1, 1, 1, 1, 1, 1, 1, 1 },
    */

    .num_wl_states                          = PCF_WL_STATES,
    .num_model_coeff                        = PCF_MODEL_COEFF,

    /* Parameters of the Cores */    
    .core_critical_temperature             	= 358.0f,
    .core_idle_power                       	= (0.4*0.5/ 0.9 / 0.9*0.4667+0.3243)*0.5+0.52804-0.12,
    .core_max_power_single                 	= 9.0f,
    .core_min_frequency                    	= 0.4f,
    .core_max_frequency                    	= 3.68f,
    .base_power_coeff						=
    #ifdef USE_MONTE_CIMONE
        { {0.3243f, 0.4667f}, {0.3243f, 0.6949f}, {0.3243f, 1.2354f}, {0.3243f, 1.6512f}, {0.3243f, 1.6001f} },
    #elif defined(USE_RHEA)
        { {0.2880f, 0.1333f}, {0.2880f, 0.52f}, {0.2880f, 1.0853f}, {0.2880f, 1.5267f}, {0.2880f, 1.4767f} },
    #else //Legacy, HIPEAC
        { {0.13333f, 0.1062f}, {0.13333f, 0.5333f}, {0.13333f, 1.2444f}, {0.13333f, 1.379894f}, {0.13333f, 1.77778f} },
    #endif

    /* Initialization Values */
    .init_core_freq                        	= 1.0f,
    .init_core_volt                        	= 0.75f,
    .init_core_ceff                         = 0.6f
    #else
    //TODO
    #endif
};

struct tasks_config_table g_TaskConfigTable =
{
    /**** Tasks Configuration *****/
    .tap_period_us			= 500,

    .tasks_period_us        = {0},
    .tasks_tap_multiplier   = {0},
    .tasks_priority         = {0}
};

struct code_config_table g_CodeConfigTable =
{
    /***** Code configuration *****/
    .use_watchdogs							= PCF_TRUE,
    .use_secure_io							= PCF_TRUE,
    .use_tests_on_numbers					= PCF_TRUE,

    .use_error_map							= PCF_TRUE
};

ctrl_config_table_t g_ControlConfigTable =
{

    //ctrl_config_param
    {
        /***** CTRL Configuration *****/
        .use_lms								= PCF_TRUE,
        .use_freq_binding						= PCF_TRUE,
        .use_quadrant_pw_budget					= PCF_TRUE,

        /***** PID Configuration ******/
        .use_pid_integral_action				= PCF_TRUE,
        .use_pid_derivative_action				= PCF_FALSE,
        .use_pid_saturation						= PCF_TRUE,
        .use_pid_ff_action						= PCF_TRUE,
        .use_pid_anti_windup_sat				= PCF_TRUE,

        .variable_voltage_solution              = VV_COUPLING_SOLUTION, //VV_MOVING_AVERAGE,

        /*** Telemetry Configuration **/
        .use_frequency_reduction_map			= PCF_TRUE
    },

    //ctrl_thermal_param
    #if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
    {    
        /* Initialization Values */
        // They have to be defined even if PID_CONFIG_x is disabled //TODO: fix this?    
        .pid_kp									= 0.5f, //2,
        .pid_ki									= 100.0f,
        .pid_kd									= 30.0f,
        .pid_dt									= 0.0005f,
        .pid_anti_windup_sat_coeff				= -0.75f,
        .pid_anti_windup_sat_up					= 0.05f,

        .pid_integral_reduction_coeff			= 1.0f,
        
        /*** Other Control Values *****/
        .pid_temp_margin					= 5.0f,
       
        .hyst_high_temp_limit					= 390.0f, //TO RESET
        .hyst_low_temp_limit					= (358.0f - 5.0f - 1.0f),

        //#ifdef USE_CTRL_FUZZY //TBD

        .fuzzy_mul                              = 2,
        .der_mul                                = 8,
        .fuzzy_temp_lim                         = {45.0f, 65.0f, 80.0f, 85.0f},
	    .fuzzy_der_lim                          = {0.0f, 0.5f, 1.0f, 2.0f},
	    .fuzzy_value_table                      = { 2,  2,	1,	0,	-1,
                                                    2,	1,	1,	0,	-1,
                                                    1,	1,	0,	-1,	-2,
                                                    1,	0,	-1,	-2,	-3,
                                                    0,	0,	-2,	-3,	-4}
    },
    #else
    //TODO
    #endif

    //ctrl_power_param
    #if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
    {
        /***** Filters and Moving Average ******/
        .moving_average_alpha                = 0.004f,

        .alpha_max_value						= 2.0f,
        .alpha_error_base_value                 = 0.5f,
        .max_pw_diff							= 0.25f
    },
    #else
    //TODO
    #endif

    //ctrl_input_param
    {},

    //ctrl_output_param
    #if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
    {
        .freq_max_gradient						= 4.0f //0.25f
    },
    #else
    //TODO
    #endif

    //ctrl_ident_param
    {
        /** Configuration **/
        //uint16_t coeff_num,
        .batch_iteration_num = 8,

        /* Values */
        // Forgetting Factor
        #if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
        .lambda = 0.948f, //0.955,
        .bound_limits =
            {{1.5f,  1.0f}, {1.5f,  1.0f}, {1.5f, 1.0f}, {1.5f, 1.0f}}
        #else
        .lambda = 1, //0.955,
        .bound_limits =
            {{1,  1}, {1,  1}, {1, 1}, {1, 1}}
        #endif
    }

};


//config_sys_t 		default_SysConfigTable;
//struct tasks_config_table  	default_TaskConfigTable;
//struct code_config_table 	default_CodeConfigTable;
//ctrl_config_table_t 	default_ControlConfigTable;


//these has to stay here
//ctrl_values_table_t     g_TasksCtrlInput;
//TODO: 
ctrl_inputs_t g_input_ctrl;
shared_task_t g_shared_task;
//ctrl_parameters_table_t g_TasksCtrlParameter;
telemetry_t             g_ChipTelemetry;

/* One Mutex for each Shared Global Variable*/
SemaphoreHandle_t       g_Sem_ChipTelemetry = NULL;
SemaphoreHandle_t       g_Sem_CtrlParameterTable = NULL;
SemaphoreHandle_t       g_Sem_CtrlInputTable = NULL;
//extern SemaphoreHandle_t g_SemReductionMap;
//extern SemaphoreHandle_t g_SemErrorMap;

uint32_t g_error_map[MAX_NUM_TASKS] = {0x0};
