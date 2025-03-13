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

//
#ifdef DEBUG_ACTIVE
#include "tgt_debug.h"
#endif
#ifdef PRINTF_ACTIVE
#include <stdlib.h>
#include <stdio.h>
#include "print_float.h"
#endif
#include <math.h>

#include "tgt_timer.h"
#include "tgt_port.h" 

/* Libraries Inclusion */
#include "sec_functions.h"
#include "ctrl_tasks.h"
#include "sec_tasks.h"
#include "pcf_tasks.h"
#include "ctrl_math.h"

//TODO all from here to end
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_system.h"
#include "cfg_control.h"
#include "cfg_constants.h"
//TODO test if I can remove this
//ADD: Need to understand the difference between "Secure Functions" and "Tasks functions"
#include "sec_globalProtectedVar.h"
#include "sec_tasks.h"
#include "imp_comms.h"

#ifdef SCMI_ACTIVE
#include "scmi_handler.h"
#endif

SemaphoreHandle_t gSem_waiting_temperature_data = NULL;

varBool_e bSecureReadConfig(void)
{
    //TODO
    /* MY DEBUG VALUES! */
    // insted here I should read from SMS memory:
    //TODO: this function should call another in system integration????
    //      or should be itself in system integration?

    // plus: this function also need to set default values

    // Important! I need to reset since MAX_NUM_TASKS can be different to the number of task.
    for (varFor task = 0; task < MAX_NUM_TASKS; task++)
    {
        g_TaskConfigTable.tasks_period_us[task]     = 0;    
        g_TaskConfigTable.tasks_tap_multiplier[task]= 0;
        g_TaskConfigTable.tasks_priority[task]      = (tskIDLE_PRIORITY);  
    }

    uint32_t period_multiplier = TASKS_PERIOD_MULTIPLIER;
    uint32_t tap_multiplier = TASKS_PERIOD_MULTIPLIER_P2;

    if (tap_multiplier < 1)
        tap_multiplier = 1;

    //TODO: this is a debug problem maybe, but make it good in all cases
    if (period_multiplier > 200)
    {
        int div = period_multiplier / 200;
        if (div <= 1) 
        {
            div = 2;
        }
        period_multiplier = 200;

        tap_multiplier *= div;
    }

    //MAIN_TASK
    g_TaskConfigTable.tasks_period_us[MAIN_TASK]                    = 5000;
    g_TaskConfigTable.tasks_tap_multiplier[MAIN_TASK]               = 0;
    g_TaskConfigTable.tasks_priority[MAIN_TASK]                     = (tskIDLE_PRIORITY + 2);  

    //PERIODIC_CONTROL_TASK
    g_TaskConfigTable.tasks_period_us[PERIODIC_CONTROL_TASK]        = 500;
    g_TaskConfigTable.tasks_tap_multiplier[PERIODIC_CONTROL_TASK]   = 0;
    g_TaskConfigTable.tasks_priority[PERIODIC_CONTROL_TASK]         = (tskIDLE_PRIORITY + 5);  

    //REAR_CONTROL_TASK
    g_TaskConfigTable.tasks_period_us[REAR_CONTROL_TASK]            = 1000*5;
    g_TaskConfigTable.tasks_tap_multiplier[REAR_CONTROL_TASK]       = 0;
    g_TaskConfigTable.tasks_priority[REAR_CONTROL_TASK]             = (tskIDLE_PRIORITY + 3);  

    //FAST_CONTROL_TASK
    g_TaskConfigTable.tasks_period_us[FAST_CONTROL_TASK]            = 125;
    g_TaskConfigTable.tasks_tap_multiplier[FAST_CONTROL_TASK]       = 0;
    g_TaskConfigTable.tasks_priority[FAST_CONTROL_TASK]             = (tskIDLE_PRIORITY + 6);  

    //COMMS_TASK
    g_TaskConfigTable.tasks_period_us[COMMS_TASK]                   = 500;
    g_TaskConfigTable.tasks_tap_multiplier[COMMS_TASK]              = 0;
    g_TaskConfigTable.tasks_priority[COMMS_TASK]                    = (tskIDLE_PRIORITY + 4);  

    //USER_DEFINED_TASK
    g_TaskConfigTable.tasks_period_us[USER_DEFINED_TASK]            = 1000*5;
    g_TaskConfigTable.tasks_tap_multiplier[USER_DEFINED_TASK]       = 0;
    g_TaskConfigTable.tasks_priority[USER_DEFINED_TASK]             = (tskIDLE_PRIORITY + 2);  

    //DEBUG_TASK
    #if (defined(DEBUG_ACTIVE) || (MEASURE_ACTIVE == 1))
    g_TaskConfigTable.tasks_period_us[DEBUG_TASK]                   = 0;
    g_TaskConfigTable.tasks_tap_multiplier[DEBUG_TASK]              = 0;
    g_TaskConfigTable.tasks_priority[DEBUG_TASK]                    = (tskIDLE_PRIORITY + 7);  
    #endif

    for (varFor task = 0; task < NUM_PERIODIC_TASKS; task++)
    {
        g_TaskConfigTable.tasks_period_us[task]                     *= period_multiplier;   
    }

    /* Compute tap */
    uint32_t l_tap_us = lMathFindArrayGCD(g_TaskConfigTable.tasks_period_us, MAX_NUM_TASKS);
    //TODO: need to check if it is 1 (so no GCD) or 0 ==> there has been an error
    //TODO: perform a check to see if tasks are in harmonic relation

    l_tap_us = usTargetTimerTapTest(l_tap_us);

    /* update on global var */
    g_TaskConfigTable.tap_period_us = l_tap_us;

    /* Compute task periodicity factors */
    #if (defined(PRINTF_ACTIVE) && defined(HRO_PRINTF))
    printf("tap: %d us - %d Hz\n\r", g_TaskConfigTable.tap_period_us, (1000000/g_TaskConfigTable.tap_period_us));
    //TODO here give the reading from FREERTOS!
    printf("Systick: %d us - %d Hz\n\r", (1000000/PCF_FREERTOS_TICK_RATE_HZ), PCF_FREERTOS_TICK_RATE_HZ);
    #endif


    for (varFor task = 0; task < NUM_PERIODIC_TASKS; task++)
    {
        //Check if the firsts NUM_PERIODIC_TASKS are actually periodic
        if (g_TaskConfigTable.tasks_period_us[task] > 0)
        {
            g_TaskConfigTable.tasks_tap_multiplier[task] = 
                g_TaskConfigTable.tasks_period_us[task] / l_tap_us * tap_multiplier;

            printf("tapp: %d\n\r", g_TaskConfigTable.tasks_tap_multiplier[task]);

            if ( (g_TaskConfigTable.tasks_period_us[task] % l_tap_us) != 0) //aka not divisible
            {
                #ifdef TAP_INTERVAL_APPROXIMATE_CEIL
                    g_TaskConfigTable.tasks_tap_multiplier[task]++;
                #endif
            }
        }
        else
        {
            //ERROR ONE TASK IS NOT PERIODIC

            //TODO: something

            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Error! One of the first %d tasks is not periodic!\n\r", NUM_PERIODIC_TASKS);
            #endif
        }
    }

    //Check if the lasts (MAX_NUM_TASKS-NUM_PERIODIC_TASKS) are actually non-periodic
    for (varFor task = NUM_PERIODIC_TASKS; task < MAX_NUM_TASKS; task++)
    {
        if (g_TaskConfigTable.tasks_period_us[task] > 0)
        {
            //ERROR this task is periodic but not considered

            //TODO: something

            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Error! One periodic task is not in the Periodic group!!\n\r");
            #endif
        }
    }
 
    /* BE CAREFUL the order of reading/modifing! */
    //g_ControlConfigTable.pid_dt = (varValue)g_TaskConfigTable.tap_period_us / 1000000.0;
    //g_ControlConfigTable.hyst_high_temp_limit = g_config_sys.core_critical_temperature - VD_ONE;
    //g_ControlConfigTable.hyst_low_temp_limit = g_config_sys.core_critical_temperature - g_ControlConfigTable.pid_temperature_margin - VD_ONE;
    //

    return PCF_TRUE;
}

varBool_e bSecureSendCoreFrequencies(varValue *i_computed_freq)
{
    //This is a function (instead of triggering directly the IRQ from the task),
    //  because in the future it may need to do other stuff, and I wanted to be
    //  separate from the task structure.

    varBool_e return_value = PCF_TRUE;

    for (int core=0; core<PCF_CORES_MAX; core++)
    {
        //TODO: GTLV?
        if (i_computed_freq[core] < g_config_sys.core_min_frequency)
        {
            i_computed_freq[core] = g_config_sys.core_min_frequency;
            varBool_e return_value = PCF_FALSE;
        }
        //TODO: GTLV?
        if (i_computed_freq[core] > g_config_sys.core_max_frequency)
        {
            i_computed_freq[core] = g_config_sys.core_max_frequency;
            varBool_e return_value = PCF_FALSE;
        }

        #ifdef SCMI_ACTIVE
        if (lowered_freq)
        {
            if (i_computed_freq[2] > 0.81)
            {
                i_computed_freq[2] = 0.8;
                lowered_freq--;
                g_shared_task.op.computed_core_frequency[2] = 0.8;
            }
        }
        #endif
    }

    if (bImpSendCoreFreq(i_computed_freq) != PCF_TRUE)
    {
        //TODO

        return_value = PCF_FALSE;
    }

    return return_value;
}

varBool_e bSecureSendDomainVoltages(varValue *i_computed_vdd)
{
    //This is a function (instead of triggering directly the IRQ from the task),
    //  because in the future it may need to do other stuff, and I wanted to be
    //  separate from the task structure.

    varBool_e return_value = PCF_TRUE;

    for (int domain=0; domain<PCF_DOMAINS_MAX; domain++)
    {
        //TODO: GTLV?
        if (i_computed_vdd[domain] < g_config_sys.voltage_table[0])
        {
            i_computed_vdd[domain] = g_config_sys.voltage_table[0];
            varBool_e return_value = PCF_FALSE;
        }
        //TODO: GTLV?
        if (i_computed_vdd[domain] > g_config_sys.voltage_table[(PCF_VOLTAGE_LEVELS-1)*3])
        {
            i_computed_vdd[domain] = g_config_sys.voltage_table[(PCF_VOLTAGE_LEVELS-1)*3];
            varBool_e return_value = PCF_FALSE;
        }
    }

    if (bImpSendDomainVoltages(i_computed_vdd) != PCF_TRUE)
    {
        //TODO

        return_value = PCF_FALSE;
    }

    return return_value;
}

varBool_e bSecureReadCoreTemperatures(varValue *o_measured_temp)
{
    //This is a function (instead of triggering directly the IRQ from the task),
    //  because in the future it may need to do other stuff, and I wanted to be
    //  separate from the task structure.

    varBool_e return_value = PCF_TRUE;

    //A: send the request
    //TODO: We don't have a priviledge mode atm
    if (bImpReadTempRequest(o_measured_temp) != PCF_TRUE)
    {
        //TODO
        return_value = PCF_FALSE;
    }

    for (varFor core = 0; core < g_config_sys.num_cores; core++)
    {
        if ( (o_measured_temp[core] < 223.0f) || (o_measured_temp[core] > 373.0f)   //TODO VALUES
                || (isnan(o_measured_temp[core])) || (isinf(o_measured_temp[core])) )
        {
            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Error temp max iada iada: "); //TODO
            printFloat(o_measured_temp[core]);
            printf("\n\r");
            #endif

            //TODO FIX
            //Fixing numerical issue
            o_measured_temp[core] = 273 + 80; 
            //#endif


            //TODO: error map, return_value ,etc.
        }

    }


    return return_value;
}

varBool_e bSecureFreertosSetup(void)
{

    varBool_e return_value = PCF_TRUE;

    uint32_t l_error_map = BM_RESET;
    //gAR_emerg = 0; //TODO

    /* One Mutex for each Shared Global Variable */
    g_Sem_ChipTelemetry 		= NULL;
    g_Sem_CtrlParameterTable 	= NULL;
    g_Sem_CtrlInputTable     	= NULL;
    //SemaphoreHandle_t g_SemReductionMap = NULL;
    //SemaphoreHandle_t g_Seml_error_map = NULL;

    /** Create Mutexes (semaphores) **/
    //No need to "Give" Semaphore after creation.
    if ( (g_Sem_ChipTelemetry = xSemaphoreCreateMutex()) == NULL )
    {
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("SemAccumulationData Mutex is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_OBJ;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-10);
    }
    if ( (g_Sem_CtrlParameterTable = xSemaphoreCreateMutex()) == NULL )
    {
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("SemCalibrationTable Mutex is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_OBJ;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-10);
    }
    if ( (g_Sem_CtrlInputTable = xSemaphoreCreateMutex()) == NULL )
    {
        #ifdef PRINTF_ACTIVE
            printf("g_Sem_CtrlInputTable Mutex is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_OBJ;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-10);
    }
    /*if (g_CodeConfigTable.use_frequency_reduction_map == PCF_TRUE)
    {
        if ( (gSemReductionMap = xSemaphoreCreateMutex()) == NULL )
        {
            #ifdef PRINTF_ACTIVE
                printf("SemReductionMap Mutex is NULL!\n\r");
            #endif

            //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
                l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_OBJ;

            return_value = PCF_FALSE;

            //TODO: do something
            exit(-10);
        }
    }
    if (g_CodeConfigTable.use_error_map == PCF_TRUE)
    {
        if ( (gSeml_error_map = xSemaphoreCreateMutex()) == NULL )
        {
            #ifdef PRINTF_ACTIVE
                printf("Seml_error_map Mutex is NULL!\n\r");
            #endif

            //Cannot notify with the Error map if the error map mutex cannot be created,
            // *** TODO: find another way to notify!!
            //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            //	l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_OBJ;

            return_value = PCF_FALSE;

            //TODO: do something
            exit(-10);
        }
        #endif
    }*/


    /*** Task Creation ***/
    for (varFor i = 0; i < MAX_NUM_TASKS; i++)
    {
        taskHandles[i] = NULL;
        g_error_map[i] = 0x0;
    }

    if( xTaskCreate(
        vPeriodicControlTask,
        "Periodic Task",
        (configMINIMAL_STACK_SIZE*8 + 1000), //TBD
        &g_error_map[PERIODIC_CONTROL_TASK],
        g_TaskConfigTable.tasks_priority[PERIODIC_CONTROL_TASK] ,
        &taskHandles[PERIODIC_CONTROL_TASK]
        ) != pdPASS )
    {
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Periodic Control Task is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_TASK;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-5);
    }

    if( xTaskCreate(
        vRearControlTask,
        "Rear Ctrl Task",
        (configMINIMAL_STACK_SIZE*2 + 200), //TBD
        &g_error_map[REAR_CONTROL_TASK],
        g_TaskConfigTable.tasks_priority[REAR_CONTROL_TASK] ,
        &taskHandles[REAR_CONTROL_TASK]
        ) != pdPASS )
    {
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Rear Control Task is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_TASK;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-5);
    }

    if( xTaskCreate(
        vCommsTask,
        "Comms Task",
        (configMINIMAL_STACK_SIZE*2 + 400), //TBD
        &g_error_map[COMMS_TASK],
        g_TaskConfigTable.tasks_priority[COMMS_TASK], 
        &taskHandles[COMMS_TASK]
        ) != pdPASS )
    {
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Communications Task is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_TASK;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-5);
    }

    if( xTaskCreate(
        vUserDefinedTask,
        "User Task",
        (configMINIMAL_STACK_SIZE + 200), //TBD
        &g_error_map[USER_DEFINED_TASK],
        g_TaskConfigTable.tasks_priority[USER_DEFINED_TASK] ,
        &taskHandles[USER_DEFINED_TASK]
        ) != pdPASS )
    {
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("User Defined Task Task is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_TASK;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-5);
    }

    if( xTaskCreate(
        vMainTask,
        "Main Task",
        (configMINIMAL_STACK_SIZE + 0), //TBD
        &g_error_map[MAIN_TASK],
        g_TaskConfigTable.tasks_priority[MAIN_TASK] ,
        &taskHandles[MAIN_TASK]
        ) != pdPASS )
    {
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Main / Management Task is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_TASK;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-5);
    }

    if( xTaskCreate(
        vFastControlTask,
        "Fast Ctrl Task",
        (configMINIMAL_STACK_SIZE *2+100), //TBD
        &g_error_map[FAST_CONTROL_TASK],
        g_TaskConfigTable.tasks_priority[FAST_CONTROL_TASK],
        &taskHandles[FAST_CONTROL_TASK]
        ) != pdPASS )
    {
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Fast Control Task is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_TASK;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-5);
    } 

    #if (defined(DEBUG_ACTIVE) || (MEASURE_ACTIVE == 1))
    if( xTaskCreate(
        vDebugTask,
        "Debug Task",
        configMINIMAL_STACK_SIZE + 200 + 1*MEASURE_N_OUTPUTS*MEASURE_N_ITERATION, //TBD
        &g_error_map[DEBUG_TASK],
        g_TaskConfigTable.tasks_priority[DEBUG_TASK],
        &taskHandles[DEBUG_TASK]
        ) != pdPASS )
    {
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            printf("Debug Task is NULL!\n\r");
        #endif

        //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
            l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_TASK;

        return_value = PCF_FALSE;

        //TODO: do something
        vTargetPcfExit(-5);
    }
    #endif


    return return_value;
}

varBool_e bSecureControlParamSetup(void)
{
    //CUGV: commonly-used-global-vars
    varFor l_num_wl_states 	= g_config_sys.num_wl_states;
    varFor l_num_core 		= g_config_sys.num_cores;
    varFor l_num_domains    = g_config_sys.num_domains;

    varBool_e return_value = PCF_TRUE;

    uint32_t l_error_map = BM_RESET;


    //TODO: should I have to call the functions to read/write? (the locks??)
    /*** Parameter_Tables and Vars Initialization ***/
    // I do it here instead of the main, because I could want to recall initialization
    //		task as a soft reset.
    /* zeroing */ //for the case num_* < MAX_NUM_*
    for (varFor i = 0; i < PCF_CORES_MAX; i++)
    {
        //telemetry
        //g_ChipTelemetry.core_avg_sensor_data[i].frequency 	= VD_ZERO;
        //g_ChipTelemetry.core_avg_sensor_data[i].voltage 	= VD_ZERO;
        //g_ChipTelemetry.core_avg_sensor_data[i].temperature = VD_ZERO;

        //g_ChipTelemetry.core_avg_estimated_power[i] 		= 0;

        //FreqRedMap
        //g_ChipTelemetry.frequency_reduction_map[i]			= BM_RESET;
    }
    for (varFor j = 0; j < PCF_WL_STATES; j++)
        for (varFor k = 0; k < PCF_MODEL_COEFF; k++)
        {
            //g_TasksCtrlInput.power_formula_coeff[j][k] 	= 0;
        }
    for (varFor q = 0; q < PCF_CORES_MAX; q++)
    {
        //g_TasksCtrlInput.quadrant_power_budget[q] 		= 0;
        //g_ChipTelemetry.quad_avg_estimated_power[q]			= 0;
    }
    //g_TasksCtrlInput.total_power_budget					= 0;
    //g_ChipTelemetry.chip_avg_estimated_power				= 0;
    //g_ChipTelemetry.power_budget_exceed_us					= 0;
    for (varFor j = 0; j < PCF_DOMAINS_MAX; j++)
    {
        //g_TasksCtrlInput.measured_power[j]              = 0;
        //g_TasksCtrlInput.voltage[j]                     = VD_ZERO;
        //g_TasksCtrlInput.max_freq[j]                    = g_config_sys.voltage_table[2];
        //g_config_sys.num_cores_per_domain[j]            = 0;
    }

    /* (non-config) Global Var Initialization */
    //CUGV
    //varValue l_init_core_freq = g_config_sys.init_core_freq;
    //varValue l_init_core_volt = g_config_sys.init_core_volt;
    //varValue l_init_core_ceff = g_config_sys.init_core_ceff;
    //varValue l_core_idle_power = g_config_sys.core_idle_power;
    //varBool_e l_use_freq_binding = g_ControlConfigTable.use_freq_binding;


    /* 1 DOMAIN */
    /*
    const uint8_t CoreQuad[PCF_CORES_MAX] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0 };
    */
    /* 4 DOMAINS */
    const uint8_t CoreQuad[PCF_CORES_MAX] =  { 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        1, 1, 1, 1, 1, 1, 1, 1, 1,
                                        2, 2, 2, 2, 2, 2, 2, 2, 2,
                                        3, 3, 3, 3, 3, 3, 3, 3, 3 };
                                        
    
    /* ALL DOMAINS */
    /*
    const uint8_t CoreQuad[PCF_CORES_MAX] = { 0, 1, 2, 3, 4, 5, 6, 7, 8,
                                        9, 10, 11, 12, 13, 14, 15, 16, 17,
                                        18, 19, 20, 21, 22, 23, 24, 25, 26,
                                        27, 28, 29, 30, 31, 32, 33, 34, 35 };
    */
    /* OG RHEA */
    /*
    const uint8_t CoreQuad[PCF_CORES_MAX] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        1, 1, 1, 1, 1, 1, 1, 1, 1,
                                        1, 1, 1, 1, 1, 1, 1, 1, 1,
                                        1, 1, 1, 1, 1, 1, 1, 1, 1 };
    */
    /* OLD stuff */
    /*                                  { 0, 0, 0, 1, 0, 0, 1, 1, 0,
                                        1, 0, 0, 1, 0, 0, 1, 1, 0,
                                        1, 0, 0, 1, 0, 0, 1, 1, 0,
                                        1, 0, 0, 1, 0, 0, 1, 1, 0 }; */

/*  
    for (varFor i = 0; i < l_num_core; i++)
    {

        g_TasksCtrlInput.target_freq[i] 				= l_init_core_freq;
        g_TasksCtrlInput.core_ceff[i]                   = l_init_core_ceff;
        #ifdef USE_INSTRUCTIONS_COMPOSITION
        g_TasksCtrlInput.perc_workload[i][0]			= 100; //g_config_sys.init_core_workload; //TODO
        for (varFor j = 1; j < l_num_wl_states; j++)
        {
            g_TasksCtrlInput.perc_workload[i][j]		= 0;
        }
        #endif
        if (l_use_freq_binding == PCF_TRUE)
        {
            //TODO
            if ((i==3)||(i==4)||(i==5)||(i==6))
                g_TasksCtrlInput.core_binding_vector[i] 	= 0;
            else
                g_TasksCtrlInput.core_binding_vector[i] 	= 0;

        }
        else
        {
            for (varFor j = 0; j < l_num_wl_states; j++)
            {
                g_TasksCtrlInput.core_binding_vector[i]	= 0;
            }
        }
        g_config_sys.core_domain_id[i]                      = CoreQuad[i];
        g_config_sys.num_cores_per_domain[CoreQuad[i]]++;
        //g_TasksCtrlInput.freq_diff_red[i]                 = VD_ZERO;

        //telemetry
        g_ChipTelemetry.core_avg_sensor_data[i].frequency 	= l_init_core_freq;
        g_ChipTelemetry.core_avg_sensor_data[i].voltage 	= l_init_core_volt;
        g_ChipTelemetry.core_avg_sensor_data[i].temperature = VD_TEMP_INIT;

        g_ChipTelemetry.core_avg_estimated_power[i] 		= l_core_idle_power;
    }
    for (varFor j = 0; j < l_num_wl_states; j++)
        for (varFor k = 0; k < PCF_MODEL_COEFF; k++)
        {
            g_TasksCtrlInput.power_formula_coeff[j][k] 	= g_config_sys.base_power_coeff[j][k];
        }
    //for (varFor j = 0; j < l_num_domains; j++)
    //{ //TODO: this initialization.
        g_TasksCtrlInput.measured_power[0]                 = l_num_core * l_core_idle_power;
        g_TasksCtrlInput.measured_power[1]                 = 40; //random number, TODO.
        //g_TasksCtrlInput.voltage[0] = 0.5;
        //g_TasksCtrlInput.voltage[1] = 0.5;
    //}
    //TODO:
    //#ifdef EPI_CONFIG_QUADRANTS
    //    float QuadrantPowerBudget[EPI_N_QUADRANTS];
    //#endif

    g_TasksCtrlInput.total_power_budget 				= g_ControlConfigTable.init_total_power_budget;
    g_TasksCtrlInput.perfmormance_level                 = 1;//80;
    g_ChipTelemetry.chip_avg_estimated_power				= g_config_sys.core_idle_power * (varValue)l_num_core;
    */


    /////NEW

    //g_input_ctrl
    //cmd
    for (varFor i = 0; i < PCF_CORES_MAX; i++)
    {
        g_input_ctrl.cmd.target_freq[i] = 1;
        g_input_ctrl.cmd.target_freq_changed[i] = 0;
       
        g_input_ctrl.cmd.core_binding_vector[i] = 0;     
        //msr
        g_input_ctrl.msr.temp.core[i] = 25+273;
        

    }
    for (varFor i = 0; i < PCF_DOMAINS_MAX; i++)
    {
        g_input_ctrl.cmd.quadrant_power_budget[i] = 100;
        //msr
        g_input_ctrl.msr.pw.domain[i] = 100;
    }
    g_input_ctrl.cmd.total_power_budget = 100;
    g_input_ctrl.cmd.perfmormance_level = 80;
    g_input_ctrl.cmd.telemetry_horizon = 10;
    //msr
    g_input_ctrl.msr.pw.total = 100;



    return return_value;
}

varBool_e bSecureFirmareSetup(void)
{
    //CUGV: commonly-used-global-vars
    varFor l_num_core 		= g_config_sys.num_cores;

    varBool_e return_value = PCF_TRUE;

    for (varFor i = 0; i < MAX_NUM_TASKS; i++)
        g_error_map[i] = BM_RESET;

    return return_value;
}
