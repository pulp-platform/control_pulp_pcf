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

#include "pcf_taskFunctions.h"

/* FreeRTOS Inclusions. */
#include <FreeRTOS.h>
#include <semphr.h>

/* Libraries Inclusion */
#include "cfg_types.h"

/* Secure */
#include "sec_globalProtectedVar.h"

/* Other */
#ifdef PRINTF_ACTIVE
#include <stdio.h>
#endif
// TODO: remove
#include <stddef.h> //size_t

/*-----------------------------------------------*/
/******************* Functions *******************/
/*-----------------------------------------------*/

/*-----------------------------------------------*/
/********************* I/Os **********************/
/*-----------------------------------------------*/

// TBD: there are 2 possibilities: create only 1 funct and pass void* and the sem, or create one for
// each Semaphore
// The first save code space and is more elegant, the second should be more secure
// I chose the second one, otherwise I had to add further security checks, like:
//      "Who called?", "The output address is correct? aka same type?", "is everything ok?"
// and moreover, look at the function: int lReadGlobalTable(void* oAddress, int cmd){
// the void* oAddress is terrible from security pow, I can overwrite everything, with
// something that is also configurable by multiple agents like these tables: UNACCEPTABLE
// But it's also true that they can do that with the second option.
// Can I change modality? For now no... //TBC

/*
varBool_e bReadGlobalVariable(void* oAddress, pcf_ctrl_struct_e i_var_name, size_t i_var_dim,
uint32_t i_caller_id){

    SemaphoreHandle_t* lock = NULL;
    TickType_t ticks_to_wait = 0; //TODO ALL SEM TIMINGS
    pcf_global_var_e global_var_index;

    //void* global_var_address = NULL; //couldn't make work this way.
    //So I did:
    telemetry_t* telemetry_address = NULL;
    ctrl_inputs_table_t* ctrl_input_address = NULL;
    ctrl_parameters_table_t* ctrl_parameter_address = NULL;

    taskENTER_CRITICAL();

    //TBU: below instead of putting the name of variable, I could create an array of addresses using
the enum as index-
    // BUT: the addresses has all different types.
    switch (i_var_name)
    {
        //TODO: add checks: if size = size, if caller_id is permitted.
        case G_CTRL_PARAMETER_TABLE:
        {
            lock = &g_Sem_CtrlParameterTable;
            ticks_to_wait = 1;
            global_var_index = G_CTRL_PARAMETER_TABLE;
            //
            ctrl_parameter_address = &g_TasksCtrlParameter;
            break;
        }
        case G_CTRL_INPUT_TABLE:
        {
            lock = &g_Sem_CtrlInputTable;
            ticks_to_wait = 1;
            global_var_index = G_CTRL_INPUT_TABLE;
            //
            ctrl_input_address = &g_TasksCtrlInput;
            break;
        }
        case G_TELEMETRY:
        {
            lock = &g_Sem_ChipTelemetry;
            ticks_to_wait = 1;
            global_var_index = G_TELEMETRY;
            //
            telemetry_address = &g_ChipTelemetry;
            break;
        }
        default:
        {
            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("%d Error, wrong type of var!\n\r", (int)i_var_name);
            #endif

            //TODO
            break;
        }

    }

    // Take the Mutex
    if (lock != NULL)
    {
        //Attempt to take the lock
        if ( xSemaphoreTake( *lock, ticks_to_wait ) == pdFALSE )
        {
            // Mutex not taken in time
            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("%d Read Mutex is not taken in time!\n\r", (int)i_var_name);
            #endif

            //TODO: do something

            return PCF_FALSE;
        }
        else
        {
            // Copy the Data

            //TODO: this is HIGHLY inefficient. FIX THIS.
            switch (i_var_name)
            {
                //TODO: add checks: if size = size, if caller_id is permitted.
                case G_CTRL_PARAMETER_TABLE:
                {
                    ctrl_parameters_table_t* casted_address = (ctrl_parameters_table_t*) oAddress;
                    *casted_address = *ctrl_parameter_address;
                    break;
                }
                case G_CTRL_INPUT_TABLE:
                {
                    ctrl_inputs_table_t* casted_address = (ctrl_inputs_table_t*) oAddress;
                    *casted_address = *ctrl_input_address;
                    break;
                }
                case G_TELEMETRY:
                {
                    telemetry_t* casted_address = (telemetry_t*) oAddress;
                    *casted_address = *telemetry_address;
                    break;
                }
                default:
                {
                    #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                        printf("%d Error, wrong type of var!\n\r", (int)i_var_name);
                    #endif

                    //TODO
                    break;
                }

            }

            taskEXIT_CRITICAL();

                // Release the Mutex
                if ( xSemaphoreGive( *lock ) == pdFALSE )
                {
                        // An error occurred. Semaphores are implemented using queues.
                        // An error can occur if there is no space on the queue to post a message
                        // indicating that the semaphore was not first obtained correctly.

                        // Mutex not Released Correctly
                        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                                printf("%d  Read Mutex is not Released Correctly!\n\r",
(int)i_var_name);
                        #endif

                        //TODO: do something

                        return PCF_FALSE;
                }
                else
                        return PCF_TRUE;
        }
    } //if (lock != NULL)
    else
    {
        //TODO
        return PCF_FALSE;
    }
}

varBool_e bWriteGlobalVariable(void* iAddress, pcf_ctrl_struct_e i_var_name, global_var_write_cmd_e
i_cmd, size_t i_var_dim, uint32_t i_caller_id){

    SemaphoreHandle_t* lock = NULL;
    TickType_t ticks_to_wait = 0; //TODO ALL SEM TIMINGS
    pcf_global_var_e global_var_index;

    //void* global_var_address = NULL; //couldn't make work this way.

    telemetry_t* telemetry_address = NULL;
    ctrl_inputs_table_t* ctrl_input_address = NULL;
    ctrl_parameters_table_t* ctrl_parameter_address = NULL;

    taskENTER_CRITICAL();

    switch (i_var_name)
    {
        //TODO: add checks: if size = size, if caller_id is permitted.
        case G_CTRL_PARAMETER_TABLE:
        {
            lock = &g_Sem_CtrlParameterTable;
            ticks_to_wait = 1;
            global_var_index = G_CTRL_PARAMETER_TABLE;
            //
            ctrl_parameter_address = &g_TasksCtrlParameter;
            break;
        }
        case G_CTRL_INPUT_TABLE:
        {
            lock = &g_Sem_CtrlInputTable;
            ticks_to_wait = 1;
            global_var_index = G_CTRL_INPUT_TABLE;
            //
            ctrl_input_address = &g_TasksCtrlInput;
            break;
        }
        case G_TELEMETRY:
        {
            lock = &g_Sem_ChipTelemetry;
            ticks_to_wait = 1;
            global_var_index = G_TELEMETRY;
            //
            telemetry_address = &g_ChipTelemetry;
            break;
        }
        default:
        {
            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("%d Error, wrong type of var!\n\r", (int)i_var_name);
            #endif

            //TODO
            break;
        }
    }

    // Take the Mutex
    if (lock != NULL)
    {
        // Take the Mutex
        if ( xSemaphoreTake( *lock, ticks_to_wait ) == pdFALSE ) //TBD: xTicksToWait
        {
                // Mutex not taken in time
                #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                        printf("%d Write Mutex is not taken in time!\n\r", (int)i_var_name);
                #endif

                //TODO: do something

                return PCF_FALSE;
        }
        else
        {

            if (i_cmd > GLOBAL_WRITE_ALL)
            {
                //TODO
            }
            else
            {
                if (i_cmd == GLOBAL_WRITE_ALL)
                {
                    // Copy the Data
                    //TODO: this is HIGHLY inefficient. FIX THIS.
                    switch (i_var_name)
                    {
                        //TODO: add checks: if size = size, if caller_id is permitted.
                        case G_CTRL_PARAMETER_TABLE:
                        {
                            ctrl_parameters_table_t* casted_address = (ctrl_parameters_table_t*)
iAddress;
                            *ctrl_parameter_address = *casted_address;
                            break;
                        }
                        case G_CTRL_INPUT_TABLE:
                        {
                            ctrl_inputs_table_t* casted_address = (ctrl_inputs_table_t*) iAddress;
                            *ctrl_input_address = *casted_address;
                            break;
                        }
                        case G_TELEMETRY:
                        {
                            telemetry_t* casted_address = (telemetry_t*) iAddress;
                            *telemetry_address = *casted_address;
                            break;
                        }
                        default:
                        {
                            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                                printf("%d Error, wrong type of var!\n\r", (int)i_var_name);
                            #endif

                            //TODO
                            break;
                        }

                    }
                }
            }

            taskEXIT_CRITICAL();

            // Release the Mutex
                if ( xSemaphoreGive( *lock ) == pdFALSE )
                {
                        // An error occurred. Semaphores are implemented using queues.
                        // An error can occur if there is no space on the queue to post a message
                        // indicating that the semaphore was not first obtained correctly.

                        // Mutex not Released Correctly
                        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                                printf("%d  Write Mutex is not Released Correctly!\n\r",
(int)i_var_name);
                        #endif

                        //TODO: do something

                        return PCF_FALSE;
                }
                else
                        return PCF_TRUE;
        }
    } //if (lock != NULL)
    else
    {
        return PCF_FALSE;
        //TODO
    }

}


uint32_t* bSecureGetErrorMapAddress(int task_id)
{
    //TBC: It is important here not to use some global definitions (e.g. PERIODIC_TASK_ID), but
    // instead we hard-code these numeric values.
    switch (task_id)
    {
        case 5367:
        {
            //Periodic Control Task:
            return (&g_error_map[PERIODIC_CONTROL_TASK]);
        }
        case 1367:
        {
            //Rear Control Task:
            return (&g_error_map[REAR_CONTROL_TASK]);
        }
        case 7321:
        {
            //Comms Task:
            return (&g_error_map[COMMS_TASK]);
        }
        default:
        {
            //TODO: Signal error!

            return NULL;
        }
    }
}

*/

/*
varBool_e bCtrlResetStruct(void *addr, pcf_ctrl_struct type)
{
    varBool_e return_value = PCF_TRUE;

    //TODO: can I manage these better?
    if (type == CTRL_MEASURES)
    {
        struct ctrl_measures  *cast_addr = addr;
        return_value &= bCtrlResetStruct(cast_addr->temp, CTRL_TEMP_MEASURES);
        return_value &= bCtrlResetStruct(cast_addr->pw, CTRL_POWER_MEASURES);
        return_value &= bCtrlResetStruct(cast_addr->perf, CTRL_PERF_MEASURES);
    }
    else if (type == CTRL_VALUE_TABLE)
    {
        struct ctrl_measures  *cast_addr = addr;
        for (varFor i = 0; i < PCF_CORES_MAX; i++)
        {
            cast_addr->target_core_power[i]         = VD_ZERO;
        }
        return_value &= bCtrlResetStruct(cast_addr->ip, CTRL_INPUT_PROCESS);
        return_value &= bCtrlResetStruct(cast_addr->th, CTRL_MOVING_AVERAGE);
        return_value &= bCtrlResetStruct(cast_addr->ma, CTRL_THERMAL);
        return_value &= bCtrlResetStruct(cast_addr->op, CTRL_OUTPUT);

        cast_addr->telemetry_counter                = 0;
    }
    else
    {
        //normal function
        switch (type)
        {
        case CTRL_COMMANDS:
            struct ctrl_commands *cast_addr = addr;
            for (varFor i = 0; i < PCF_CORES_MAX; i++)
            {
                cast_addr->target_freq[i]           = VD_ZERO;
                cast_addr->core_binding_vector[i]   = 0;
            }
            for (varFor i = 0; i < PCF_DOMAINS_MAX; i++)
            {
                cast_addr->quadrant_power_budget[i] = VD_ZERO;
            }
            cast_addr->total_power_budget           = 0;
            cast_addr->perfmormance_level           = 0;
            cast_addr->telemetry_horizon            = 0;

            break;

        case CTRL_TEMP_MEASURES:
            struct temperature_measures *cast_addr = addr;
            for (varFor i = 0; i < PCF_CORES_MAX; i++)
            {
                cast_addr->core[i]                 = VD_ZERO;
            }

            break;
        case CTRL_POWER_MEASURES:
            struct power_measures *cast_addr = addr;
            for (varFor i = 0; i < PCF_DOMAINS_MAX; i++)
            {
                cast_addr->domain[i]                = VD_ZERO;
            }

            break;
        case CTRL_PERF_MEASURES:
            struct performance_measures *cast_addr = addr;
            for (varFor i = 0; i < PCF_CORES_MAX; i++)
            {
                #ifdef USE_INSTRUCTIONS_COMPOSITION
                for (varFor j = 0; j < PCF_WL_STATES; j++)
                {
                    cast_addr->perc[i][j]           = 0;
                }
                #endif
                cast_addr->ceff[i]                  = VD_ZERO;
            }

            break;

        case CTRL_INPUT_PROCESS:
            struct ctrl_input_process *cast_addr = addr;

            //Power Adaptation
            cast_addr->prev_power_budget            = VD_ZERO;
            cast_addr->power_budget_changed         = PCF_FALSE;
            cast_addr->total_power_adaptation_term  = VD_ZERO;

            break;
        case CTRL_MOVING_AVERAGE:
            struct ctrl_moving_average *cast_addr = addr;

            cast_addr->alpha                        = VD_ZERO;
            cast_addr->alpha_counter                = 0;
            for (varFor i = 0; i < PCF_CORES_MAX; i++)
            {
                cast_addr->og_freq[i]               = VD_ZERO;
                cast_addr->freq_point[i]            = VD_ZERO;
            }

            break;
        case CTRL_THERMAL:
            struct ctrl_thermal *cast_addr = addr;

            //Control
            for (varFor i = 0; i < PCF_CORES_MAX; i++)
            {
                cast_addr->ctrl_cmd[i]              = VD_ZERO;
                cast_addr->hyst_thresh_reached[i]   = PCF_FALSE;
                //PID
                cast_addr->_pid_integral_error[i]   = VD_ZERO;
                cast_addr->_pid_previous_error[i]   = VD_ZERO;
            }

            break;
        case CTRL_OUTPUT:
            struct ctrl_output *cast_addr = addr;

            for (varFor i = 0; i < PCF_CORES_MAX; i++)
            {
                cast_addr->computed_core_frequency[i] = VD_ZERO;
            }
            for (varFor i = 0; i < PCF_DOMAINS_MAX; i++)
            {
                cast_addr->computed_domain_voltage[i] = VD_ZERO;
                cast_addr->store_max_freq[i]        = VD_ZERO;
            }

            break;

        default:
            return_value = PCF_FALSE;
            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("[Firmware] Error! Unknown struct id: %d.\r\n", type);
            #endif

            break;
        }
    } //if

    return return_value;

}

varBool_e bCtrlInitStruct(void *addr, pcf_ctrl_struct type)
{
    varBool_e return_value = PCF_TRUE;

    //TODO: can I manage these better?
    if (type == CTRL_MEASURES)
    {
        struct ctrl_measures  *cast_addr = addr;
        return_value &= bCtrlInitStruct(cast_addr->temp, CTRL_TEMP_MEASURES);
        return_value &= bCtrlInitStruct(cast_addr->pw, CTRL_POWER_MEASURES);
        return_value &= bCtrlInitStruct(cast_addr->perf, CTRL_PERF_MEASURES);
    }
    else if (type == CTRL_VALUE_TABLE)
    {
        struct ctrl_measures  *cast_addr = addr;
        for (varFor i = 0; i < g_SysConfigTable.num_cores; i++)
        {
            cast_addr->target_core_power[i]         = g_SysConfigTable.init_core_freq;
        }
        return_value &= bCtrlInitStruct(cast_addr->ip, CTRL_INPUT_PROCESS);
        return_value &= bCtrlInitStruct(cast_addr->th, CTRL_MOVING_AVERAGE);
        return_value &= bCtrlInitStruct(cast_addr->ma, CTRL_THERMAL);
        return_value &= bCtrlInitStruct(cast_addr->op, CTRL_OUTPUT);

        cast_addr->telemetry_counter                = 500000 /
g_TaskConfigTable.tasks_period_us[PERIODIC_CONTROL_TASK];
    }
    else
    {
        //normal function
        switch (type)
        {
        case CTRL_COMMANDS:
            struct ctrl_commands *cast_addr = addr;
            for (varFor i = 0; i < g_SysConfigTable.num_cores; i++)
            {
                cast_addr->target_freq[i]           = g_TaskConfigTable.init_core_freq;
                cast_addr->core_binding_vector[i]   = 0;
            }
            for (varFor i = 0; i < g_SysConfigTable.num_domains; i++)
            {
                cast_addr->quadrant_power_budget[i] = g_TaskConfigTable.num_cores_per_domain[i] *
                    g_TaskConfigTable.core_max_power_single / 2;
            }
            cast_addr->total_power_budget           = g_SysConfigTable.num_cores *
                    g_TaskConfigTable.core_max_power_single / 2;
            cast_addr->perfmormance_level           = 80;
            cast_addr->telemetry_horizon            = 500000 /
g_TaskConfigTable.tasks_period_us[PERIODIC_CONTROL_TASK];

            break;

        case CTRL_TEMP_MEASURES:
            struct temperature_measures *cast_addr = addr;
            for (varFor i = 0; i < g_SysConfigTable.num_cores; i++)
            {
                cast_addr->core[i]                 = VD_TEMP_INIT;
            }

            break;
        case CTRL_POWER_MEASURES:
            struct power_measures *cast_addr = addr;
            for (varFor i = 0; i < g_SysConfigTable.num_domains; i++)
            {
                cast_addr->domain[i]                = g_TaskConfigTable.num_cores_per_domain[i] *
                    g_TaskConfigTable.core_idle_power;
            }

            break;
        case CTRL_PERF_MEASURES:
            struct performance_measures *cast_addr = addr;
            for (varFor i = 0; i < g_SysConfigTable.num_cores; i++)
            {
                #ifdef USE_INSTRUCTIONS_COMPOSITION
                cast_addr->perc[i][0]           = 60;
                cast_addr->perc[i][1]           = 40;
                for (varFor j = 2; j < g_SysConfigTable.num_wl_states; j++)
                {
                    cast_addr->perc[i][j]           = 0;
                }
                #endif
                cast_addr->ceff[i]                  = g_TaskConfigTable.init_core_ceff;
            }

            break;

        case CTRL_INPUT_PROCESS:
            struct ctrl_input_process *cast_addr = addr;

            //Power Adaptation
            cast_addr->prev_power_budget            = g_SysConfigTable.num_cores *
                    g_TaskConfigTable.core_max_power_single / 2;
            cast_addr->power_budget_changed         = PCF_FALSE;
            cast_addr->total_power_adaptation_term  = VD_ZERO;

            break;
        case CTRL_MOVING_AVERAGE:
            struct ctrl_moving_average *cast_addr = addr;

            cast_addr->alpha                        = 0.004f;
            cast_addr->alpha_counter                = 0;
            for (varFor i = 0; i < g_SysConfigTable.num_cores; i++)
            {
                cast_addr->og_freq[i]               = g_TaskConfigTable.init_core_freq;
                cast_addr->freq_point[i]            = g_TaskConfigTable.init_core_freq;
            }

            break;
        case CTRL_THERMAL:
            struct ctrl_thermal *cast_addr = addr;

            //Control
            for (varFor i = 0; i < g_SysConfigTable.num_cores; i++)
            {
                cast_addr->ctrl_cmd[i]              = VD_ZERO;
                cast_addr->hyst_thresh_reached[i]   = PCF_FALSE;
                //PID
                cast_addr->_pid_integral_error[i]   = VD_ZERO;
                cast_addr->_pid_previous_error[i]   = VD_ZERO;
            }

            break;
        case CTRL_OUTPUT:
            struct ctrl_output *cast_addr = addr;

            for (varFor i = 0; i < g_SysConfigTable.num_cores; i++)
            {
                cast_addr->computed_core_frequency[i] = g_SysConfigTable.init_core_freq;
            }
            for (varFor i = 0; i < g_SysConfigTable.num_domains; i++)
            {
                cast_addr->computed_domain_voltage[i] = g_SysConfigTable.init_core_volt;
                cast_addr->store_max_freq[i]        = g_SysConfigTable.init_core_freq;
            }

            break;

        default:
            return_value = PCF_FALSE;
            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("[Firmware] Error! Unknown struct id: %d.\r\n", type);
            #endif

            break;
        }
    } //if

    return return_value;

}

void vCtrlResetTelem(void *addr, pcf_ctrl_struct type)
{

}

void vCtrlInitTelem(void *addr, pcf_ctrl_struct type)
{

}

*/
