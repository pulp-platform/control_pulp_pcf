
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


sys_config_table_t g_SysConfigTable =
{
    .num_core                              	= MAX_NUM_CORE,
    .num_quad	                         	= MAX_NUM_QUAD,
    .num_pw_domains                         = SYS_PW_DOMAIN_NUM,
    .num_chip_proc                         	= 1,
    .num_socket_sys                        	= 1,
    .num_voltage_levels                     = 15,
    .voltage_table                          =
    /*
        {0.50f, 0.40f, 1.35f, 
         0.55f, 1.00f, 1.60f, 
         0.60f, 1.35f, 1.80f,
         0.65f, 1.60f, 2.00f,
         0.70f, 1.80f, 2.20f,
         0.75f, 2.00f, 2.40f,
         0.80f, 2.20f, 2.60f,
         0.85f, 2.40f, 2.75f,
         0.90f, 2.60f, 2.90f,
         0.95f, 2.75f, 3.05f,
         1.00f, 2.90f, 3.20f,
         1.05f, 3.05f, 3.35f,
         1.10f, 3.20f, 3.45f,
         1.15f, 3.35f, 3.55f,
         1.20f, 3.45f, 3.66f
     },
     */
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

    /* Parameters of the Cores */
    #if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
    .core_critical_temperature             	= 358.0f,
    .core_idle_power                       	= (0.4*0.5/0.9/0.9*0.4667+0.3243)*0.5+0.52804  + 0.3,
    .core_max_power_single                 	= 9.0f,
    .core_min_frequency                    	= 0.4f,
    .core_max_frequency                    	= 3.66f,
    .core_wl_states_num                    	= SYS_CORE_WL_STATES,
    .base_power_coeff						=
    #ifdef USE_MONTE_CIMONE
        { {0.3243f, 0.4667f}, {0.3243f, 0.6949f}, {0.3243f, 1.2354f}, {0.3243f, 1.6512f}, {0.3243f, 1.6001f} },
    #else //Legacy, HIPEAC
        { {0.13333f, 0.1062f}, {0.13333f, 0.5333f}, {0.13333f, 1.2444f}, {0.13333f, 1.379894f}, {0.13333f, 1.77778f} },
    #endif

    /* Initialization Values */
    .init_core_freq                        	= 1.0f,
    .init_core_volt                        	= 0.75f,
    .init_core_workload                    	= 0.5333f,
    .init_core_ceff                         = 0.6f
    #else
    .core_critical_temperature             	= 358,
    .core_idle_power                       	= 0,
    .core_max_power_single                 	= 4,
    .core_min_frequency                    	= 0,
    .core_max_frequency                    	= 3,
    .core_wl_states_num                    	= SYS_CORE_WL_STATES,
    .base_power_coeff						=
        { {0, 0}, {0, 1}, {1, 1}, {0, 2} },

    /* Initialization Values */
    .init_core_freq                        	= 1,
    .init_core_volt                        	= 1,
    .init_core_workload                    	= 1,
    .init_core_ceff                         = 1
    #endif
};

tasks_config_table_t g_TaskConfigTable =
{
    /**** Tasks Configuration *****/
    .tap_period_us					= 500,
    /*
    .periodic_ctrl_period_us		= 500,
    .task_os_period_us				= 1000,
    */
    .tasks_period_us = {0},
    .tasks_tap_multiplier = {0},
    .tasks_priority = {0},

    // Priority
    /*
    .periodic_ctrl_task_priority			= (tskIDLE_PRIORITY + 5),
    .rear_ctrl_task_priority				= (tskIDLE_PRIORITY + 4),
    .fast_ctrl_task_priority                = (tskIDLE_PRIORITY + 6),
    .comms_task_priority					= (tskIDLE_PRIORITY + 3),
    .main_task_priority			            = 7, //(configMAX_PRIORITIES - 1);
    .user_defined_task_priority             = (tskIDLE_PRIORITY + 3),
    #if (defined(DEBUG_ACTIVE) || (MEASURE_ACTIVE == 1))
        .debug_task_priority				= (tskIDLE_PRIORITY + 2),
    #endif
    */

    /*** Telemetry Configuration **/
    .telemetry_polling_period_us			= 1000
};
//TODO
//g_TaskConfigTable.telemetry_polling_period_us = g_TaskConfigTable.tap_period_us * g_TaskConfigTable.task_os_periodicity_mul_factor / 1000;


code_config_table_t g_CodeConfigTable =
{
    /***** Code configuration *****/
    .use_watchdogs							= PCF_TRUE,
    .use_secure_io							= PCF_TRUE,
    .use_tests_on_numbers					= PCF_TRUE,

    /*** Telemetry Configuration **/
    .use_frequency_reduction_map			= PCF_TRUE,
    .use_error_map							= PCF_TRUE
};

ctrl_config_table_t g_ControlConfigTable =
{

    /***** CTRL Configuration *****/
    .use_lms								= PCF_TRUE,
    .use_freq_binding						= PCF_TRUE,
    .use_quadrant_pw_budget					= PCF_TRUE,
    .variable_voltage_solution              = VV_MOVING_AVERAGE,

    /***** PID Configuration ******/
    .use_pid_integral_action				= PCF_TRUE,
    .use_pid_derivative_action				= PCF_FALSE,
    .use_pid_saturation						= PCF_TRUE,
    .use_pid_ff_action						= PCF_TRUE,
    .use_pid_anti_windup_sat				= PCF_TRUE,

    #if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
    /***** Filters and Moving Average ******/
    .vv_moving_average_alpha                        = 0.004f,

    /* Initialization Values */
    // They have to be defined even if PID_CONFIG_x is disabled //TODO: fix this?    
    .pid_kp									= 0.5f, //2,
    .pid_ki									= 100.0f,
    .pid_kd									= 30.0f,
    .pid_dt									= 0.0005f,
    .pid_anti_windup_sat_coeff				= -0.75f,
    .pid_anti_windup_sat_up					= 0.05f,

    /*** Other Control Values *****/
    .pid_temperature_margin					= 5.0f,
    .freq_max_gradient						= 0.25f,
    .alpha_max_value						= 2.0f,
    .alpha_error_base_value                 = 0.5f,
    .hyst_high_temp_limit					= 390.0f, //TO RESET
    .hyst_low_temp_limit					= (358.0f - 5.0f - 1.0f),
    .max_pw_diff							= 0.25f,
    .pid_integral_reduction_coeff			= 1.0f,

    /* Initialization Values */
    .init_total_power_budget               	= 80.5f
    #else
    /***** Filters and Moving Average ******/
    .vv_moving_average_alpha                = 1,

    .pid_kp									= 1, //2,
    .pid_ki									= 10,
    .pid_kd									= 3,
    .pid_dt									= 1,
    .pid_anti_windup_sat_coeff				= -1,
    .pid_anti_windup_sat_up					= 1,

    /*** Other Control Values *****/
    .pid_temperature_margin					= 10,
    .freq_max_gradient						= 1,
    .alpha_max_value						= 2,
    .alpha_error_base_value                 = 1,
    .hyst_high_temp_limit					= 390,
    .hyst_low_temp_limit					= (358 - 10 - 1),
    .max_pw_diff							= 1,
    .pid_integral_reduction_coeff			= 1,

    /* Initialization Values */
    .init_total_power_budget               	= 22
    #endif

};

ctrl_lms_config_table_t g_LmsConfigTable =
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
};


sys_config_table_t 		default_SysConfigTable;
tasks_config_table_t 	default_TaskConfigTable;
code_config_table_t 	default_CodeConfigTable;
ctrl_lms_config_table_t default_LmsConfigTable;
ctrl_config_table_t 	default_ControlConfigTable;


//these has to stay here
ctrl_inputs_table_t g_TasksCtrlInput;
ctrl_parameters_table_t g_TasksCtrlParameter;
telemetry_t g_ChipTelemetry;

/* One Mutex for each Shared Global Variable*/
SemaphoreHandle_t g_Sem_ChipTelemetry = NULL;
SemaphoreHandle_t g_Sem_CtrlParameterTable = NULL;
SemaphoreHandle_t g_Sem_CtrlInputTable = NULL;
//extern SemaphoreHandle_t g_SemReductionMap;
//extern SemaphoreHandle_t g_SemErrorMap;

uint32_t g_error_map[MAX_NUM_TASKS] = {0x0};
