
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




/* FreeRTOS Inclusions. */


/* Libraries Inclusion */
#include "imp_comms.h"

#include "cfg_types.h"
#include "cfg_system.h"
#include "cfg_constants.h"

#include "imp_addresses.h"

//todo remove
#include "cfg_firmware.h"

/* Others */
//TODO: DELETE:
#include <stdint.h>
#ifdef PRINTF_ACTIVE
#include <stdio.h>
#include "print_float.h"
#endif

//Uncomment to enable Fixed addresses data fetch
//#define MEMORY_FIXED_FETCH

uint32_t l_error_map = BM_RESET;

int intdata_2b_stored[MAX_NUM_CORE] = {0};
float floatdata_2b_stored[MAX_NUM_CORE] = {0};

varBool_e bImpCommsInit(void)
{
    varBool_e return_value = PCF_TRUE;
    //TBU

	/* Lock Init */
    /*
    if ( (gSem_waiting_temperature_data = xSemaphoreCreateMutex()) == NULL )
	{
		#ifdef PRINTF_ACTIVE
			printf("gSem_waiting_temperature_data Sem is NULL!\n");
		#endif

		l_error_map |= BM_ERROR_FREERTOS_MEMORY_ALLOCATION_OBJ;
        return_value = PCF_FALSE;
		//TODO: do something

		exit(-10);
	}
    */

    return return_value;
}

varBool_e bImpSendCoreFreq(varValue *i_computed_freq)
{
	//ATTENTION: this function is called INSIDE an IRQ!
	//KEEP IT SHORT and no STRANGE CALLS.

    varBool_e return_value = PCF_TRUE;
    varFor l_num_core = g_SysConfigTable.num_core;

    #ifndef MEMORY_FIXED_FETCH
    //static uint32_t l_iteration_times = 0;
    static varValue* l_prev_iteration_address = (varValue*)IMP_ADR_OUT_FIRST_CORE_FREQ;
    #endif

    varValue* mem_address = NULL;
    #ifdef MEMORY_FIXED_FETCH
	mem_address = (varValue*)IMP_ADR_OUT_FIRST_CORE_FREQ;
    #else
    mem_address = l_prev_iteration_address;
    #endif
	size_t address_num_of_bytes = sizeof(varValue);
    //TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    //#ifndef MEMORY_FIXED_FETCH
    //mem_address += l_iteration_times * l_num_core * address_num_of_bytes;

    #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //printf("Computed Freq:  ");
    #endif

    for (varFor i = 0; i < l_num_core; i++)
    {
        *mem_address = i_computed_freq[i];
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
        //printf(" %d: ", i);
        //printFloat(i_computed_freq[i]);
        #endif

        mem_address += address_num_of_bytes;

        if (mem_address > (varValue*)IMP_ADR_OUT_CORE_FREQ_LAST)
        {
            return_value = PCF_FALSE;
            mem_address = (varValue*)IMP_ADR_OUT_FIRST_CORE_FREQ;
            if (i < (l_num_core -1))
                i = -1;
        }
    }

    #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //printf("\n\r");
    #endif

    #ifndef MEMORY_FIXED_FETCH
	l_prev_iteration_address = mem_address;
    #endif

    // TODO: Notify end of writes

    return return_value;
}


varBool_e bImpSendDomainVoltages(varValue *i_computed_voltage)
{
    //ATTENTION: this function is called INSIDE an IRQ!
    //KEEP IT SHORT and no STRANGE CALLS.

    varBool_e return_value = PCF_TRUE;
    varFor l_num_domains = g_SysConfigTable.num_pw_domains;

    #ifndef MEMORY_FIXED_FETCH
    //static uint32_t l_iteration_times = 0;
    static varValue* l_prev_iteration_address = (varValue*)IMP_ADR_OUT_FIRST_CORE_FREQ;
    #endif

    varValue* mem_address = NULL;
    #ifdef MEMORY_FIXED_FETCH
    mem_address = (varValue*)IMP_ADR_OUT_FIRST_CORE_FREQ;
    #else
    mem_address = l_prev_iteration_address;
    #endif
    size_t address_num_of_bytes = sizeof(varValue);
    //TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    //#ifndef MEMORY_FIXED_FETCH
    //mem_address += l_iteration_times * l_num_domains * address_num_of_bytes;

    #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //printf("Computed vdd:  ");
    #endif

    for (varFor i = 0; i < l_num_domains; i++)
    {
        *mem_address = i_computed_voltage[i];
        #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
        //printf(" %d: ", i);
        //printFloat(i_computed_voltage[i]);
        #endif

        mem_address += address_num_of_bytes;

        if (mem_address > (varValue*)IMP_ADR_OUT_CORE_FREQ_LAST)
        {
            return_value = PCF_FALSE;
            mem_address = (varValue*)IMP_ADR_OUT_FIRST_CORE_FREQ;
            if (i < (l_num_domains -1))
                i = -1;
        }
    }

    #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //printf("\n\r");
    #endif

    #ifndef MEMORY_FIXED_FETCH
    l_prev_iteration_address = mem_address;
    #endif

    // TODO: Notify end of writes

    return return_value;
}

varBool_e bImpReadTempRequest(varValue *o_measured_temp)
{
    varBool_e return_value = PCF_TRUE;
    varFor l_num_core = g_SysConfigTable.num_core;

    #ifndef MEMORY_FIXED_FETCH
    //static uint32_t l_iteration_times = 0;
    static varValue* l_prev_iteration_address = (varValue*)IMP_ADR_IN_FIRST_CORE_TEMP;
    #endif

    varValue* mem_address = NULL;
    #ifdef MEMORY_FIXED_FETCH
	mem_address = (varValue*)IMP_ADR_IN_FIRST_CORE_TEMP;
    #else
    mem_address = l_prev_iteration_address;
    #endif
	size_t address_num_of_bytes = sizeof(varValue);
    //TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    //#ifndef MEMORY_FIXED_FETCH
    //mem_address += l_iteration_times * l_num_core * address_num_of_bytes;

    for (varFor i = 0; i < l_num_core; i++)
    {
        varValue data = *mem_address;
        if (data == 0xCAFECAFE)
        {
            return_value = PCF_FALSE;
            mem_address= (varValue*)IMP_ADR_IN_FIRST_CORE_TEMP;
            data = *mem_address;
            i=0;
        }
        o_measured_temp[i] = data;

        mem_address += address_num_of_bytes;

        if (mem_address > (varValue*)IMP_ADR_IN_CORE_TEMP_LAST)
        {
            return_value = PCF_FALSE;
            mem_address = (varValue*)IMP_ADR_IN_FIRST_CORE_TEMP;
            if (i < (l_num_core -1))
                i = -1;
        }
    }

    #ifndef MEMORY_FIXED_FETCH
    l_prev_iteration_address = mem_address;
    #endif

    // TODO: Notify end of writes

    return return_value;
}
/*
varBool_e bImpReadCoreTemp(varValue *o_measured_temp)
{
    varBool_e return_value = PCF_TRUE;
    varFor l_num_core = g_SysConfigTable.num_core;

	varValue* mem_address = TILE_ADR_FIRST_CORE_TEMP_2;

	for (varFor i = 0; i < l_num_core; i++)
    {
        o_measured_temp[i] = *mem_address;

        mem_address += VAR_VALUE_BYTES_NUM;
    }


    return return_value;
}*/

varBool_e bImpReadInputParameters(ctrl_inputs_table_t* i_input_table)
{
    varBool_e return_value = PCF_TRUE;
    varFor l_num_core = g_SysConfigTable.num_core;

    #ifndef MEMORY_FIXED_FETCH
    //static uint32_t l_iteration_times = 0;
    static varValue* l_prev_iteration_address_ft[MAX_NUM_CORE] = {NULL};
    static varValue* l_prev_iteration_address_pwb = (varValue*)IMP_ADR_CMD_POWER_BUDGET;
    static int* l_prev_iteration_address_bind = (int*)IMP_ADR_CMD_FIRST_CORE_BINDINGS;

    static int init = 1;

    if (init)
    {
        for (varFor i = 0; i < l_num_core; i++)
            l_prev_iteration_address_ft[i] = (varValue*)(IMP_ADR_CMD_FIRST_CORE_FREQ_T + 4*2*i/* * address_num_of_bytes */);

        init = 0;
    }
    #endif

    //TBD: is it better fetch all num_core 1-type-of-data or better for each num_core fetch all data?
    //This is not the most efficient way of doing it, but not important for the date paper
    varValue* mem_address_ft = NULL;
    varValue* mem_address_pwb = NULL;
    int* mem_address_bind = NULL;

    #ifdef MEMORY_FIXED_FETCH
	mem_address_ft = (varValue*)IMP_ADR_CMD_FIRST_CORE_FREQ_T;
    mem_address_pwb = (varValue*)IMP_ADR_CMD_POWER_BUDGET;
    mem_address_bind = (int*)IMP_ADR_CMD_FIRST_CORE_BINDINGS;
    #else
    //mem_address_ft = l_prev_iteration_address_ft[0];
    mem_address_pwb = l_prev_iteration_address_pwb;
    mem_address_bind = l_prev_iteration_address_bind;
    static int l_counter_times_ft[MAX_NUM_CORE] = {0};
    static int l_counter_times_pwb = 0;
    static int l_counter_times_bind = 0;
    varValue* mem_address_times = NULL;
    int times = 0;
    #endif

    //I won't fix this ugly thing since it is broken anyway
	size_t address_num_of_bytes = sizeof(varValue);
    //TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    //#ifndef MEMORY_FIXED_FETCH
    //mem_address += l_iteration_times * l_num_core * address_num_of_bytes;

    /* Target Frequency */
    for (varFor i = 0; i < l_num_core; i++)
    {
        #ifndef MEMORY_FIXED_FETCH
        mem_address_ft = l_prev_iteration_address_ft[i];
        //printf("main FT add: %#010x\n\r", mem_address_ft);
        mem_address_times = mem_address_ft + address_num_of_bytes;
        //printf("times FT add: %#010x\n\r", mem_address_times);
        times = (int)*mem_address_times;
        if ((times == -1) || (l_counter_times_ft[i] >= times))
        {
            l_counter_times_ft[i] = 0;
            if (times != -1)
                mem_address_ft += 2*l_num_core*address_num_of_bytes;
        }
        else
        {
            l_counter_times_ft[i]++;
        }
        #endif

        varValue data = *mem_address_ft;
        if (data == 0xCAFECAFE)
        {
            return_value = PCF_FALSE;
            mem_address_ft= (varValue*)IMP_ADR_CMD_FIRST_CORE_FREQ_T;
            data = *mem_address_ft;
            i=0;
        }

        i_input_table->target_freq[i] = data;
        #ifdef MEMORY_FIXED_FETCH
        mem_address_ft += address_num_of_bytes;
        mem_address_ft += address_num_of_bytes;
        #endif

        if (mem_address_ft > (varValue*)IMP_ADR_CMD_CORE_FREQ_T_LAST)
        {
            return_value = PCF_FALSE;
            mem_address_ft = (varValue*)IMP_ADR_CMD_FIRST_CORE_FREQ_T;
            if (i < (l_num_core -1))
                i = -1;
        }

        #ifndef MEMORY_FIXED_FETCH
        l_prev_iteration_address_ft[i] = mem_address_ft;
        #endif
    }

    /* Power Budget */
    #ifndef MEMORY_FIXED_FETCH
    mem_address_pwb = l_prev_iteration_address_pwb;
    mem_address_times = mem_address_pwb + address_num_of_bytes;
    times = (int)*mem_address_times;
    if ((times == -1) || (l_counter_times_pwb > times)) //here strictly
    {
        l_counter_times_pwb = 0;
        if (times != -1)
        {
            l_prev_iteration_address_pwb += 2*address_num_of_bytes;
            mem_address_pwb = l_prev_iteration_address_pwb;
        }
    }
    else
    {
        l_counter_times_pwb++;
    }
    #endif

    varValue datap = *mem_address_pwb;
    if (datap == 0xCAFECAFE)
    {
        return_value = PCF_FALSE;
        mem_address_pwb= (varValue*)IMP_ADR_CMD_POWER_BUDGET;
        datap = *mem_address_pwb;
    }

    i_input_table->total_power_budget = datap;

    if (mem_address_pwb > (varValue*)IMP_ADR_CMD_POWER_BUDGET_LAST)
    {
        return_value = PCF_FALSE;
        mem_address_pwb = (varValue*)IMP_ADR_CMD_POWER_BUDGET;
    }


    /* Bindings */
    #ifndef MEMORY_FIXED_FETCH
    mem_address_bind = l_prev_iteration_address_bind;
    mem_address_times = mem_address_bind + address_num_of_bytes;
    times = (int)*mem_address_times;
    if ((times == -1) || (l_counter_times_bind > times)) //here strictly
    {
        l_counter_times_bind = 0;
        if (times != -1)
        {
            l_prev_iteration_address_bind += 2*l_num_core*address_num_of_bytes;
            mem_address_bind = l_prev_iteration_address_bind;
        }
    }
    else
    {
        l_counter_times_bind++;
    }
    #endif

    for (varFor i = 0; i < l_num_core; i++)
    {
        int data = *mem_address_bind;
        if (data == 0xCAFECAFE)
        {
            return_value = PCF_FALSE;
            mem_address_bind= (int*)IMP_ADR_CMD_FIRST_CORE_BINDINGS;
            data = *mem_address_bind;
            i=0;
        }

        i_input_table->core_binding_vector[i] = (varCoreBinding)data;
        mem_address_bind += address_num_of_bytes;
        mem_address_bind += address_num_of_bytes;


        if (mem_address_bind > (int*)IMP_ADR_CMD_CORE_BINDINGS_LAST)
        {
            return_value = PCF_FALSE;
            mem_address_bind = (int*)IMP_ADR_CMD_FIRST_CORE_BINDINGS;
            if (i < (l_num_core -1))
                i = -1;
        }

    }

    // TODO: Notify end of writes

    return return_value;

}

varBool_e bImpReadInstructionComposition(ctrl_inputs_table_t* i_input_table)
{
    varBool_e return_value = PCF_TRUE;
    varFor l_num_core = g_SysConfigTable.num_core;

    #ifndef MEMORY_FIXED_FETCH
    //static uint32_t l_iteration_times = 0;
    static varValue* l_prev_iteration_address = (varValue*)IMP_ADR_IN_FIRST_CORE_INSTR;
    #endif

    varValue* mem_address = NULL;
    #ifdef MEMORY_FIXED_FETCH
	mem_address = (varValue*)IMP_ADR_IN_FIRST_CORE_INSTR;
    #else
    mem_address = l_prev_iteration_address;
    #endif
	size_t address_num_of_bytes = sizeof(varValue);
    //TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    //#ifndef MEMORY_FIXED_FETCH
    //mem_address += l_iteration_times * l_num_core * address_num_of_bytes;

    for (varFor i = 0; i < l_num_core; i++)
    {
        varValue data = *mem_address;
        if (data == 0xCAFECAFE)
        {
            return_value = PCF_FALSE;
            mem_address= (varValue*)IMP_ADR_IN_FIRST_CORE_INSTR;
            data = *mem_address;
            i=0;
        }
        i_input_table->core_ceff[i] = data;

        mem_address += address_num_of_bytes;

        if (mem_address > (varValue*)IMP_ADR_IN_CORE_INSTR_LAST)
        {
            return_value = PCF_FALSE;
            mem_address = (varValue*)IMP_ADR_IN_FIRST_CORE_INSTR;
            if (i < (l_num_core -1))
                i = -1;
        }
    }

    #ifndef MEMORY_FIXED_FETCH
    l_prev_iteration_address = mem_address;
    #endif

    // TODO: Notify end of writes

    return return_value;
}

varBool_e bImpReadPowerMeasure(ctrl_inputs_table_t* i_input_table)
{
    varBool_e return_value = PCF_TRUE;
    varFor l_num_core = g_SysConfigTable.num_core;

    #ifndef MEMORY_FIXED_FETCH
    //static uint32_t l_iteration_times = 0;
    static varValue* l_prev_iteration_address = (varValue*)IMP_ADR_IN_POWER_CPU;
    #endif

    varValue* mem_address = NULL;
    #ifdef MEMORY_FIXED_FETCH
	mem_address = (varValue*)IMP_ADR_IN_POWER_CPU;
    #else
    mem_address = l_prev_iteration_address;
    #endif
	size_t address_num_of_bytes = sizeof(varValue);
    //TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    //#ifndef MEMORY_FIXED_FETCH
    //mem_address += l_iteration_times * l_num_core * address_num_of_bytes;

    varValue data = *mem_address;
    if (data == 0xCAFECAFE)
    {
        return_value = PCF_FALSE;
        mem_address= (varValue*)IMP_ADR_IN_POWER_CPU;
        data = *mem_address;
    }
    i_input_table->measured_power[0] = data;

    mem_address += address_num_of_bytes;

    if (mem_address > (varValue*)IMP_ADR_IN_POWER_CPU_LAST)
    {
        return_value = PCF_FALSE;
        mem_address = (varValue*)IMP_ADR_IN_POWER_CPU;
    }

    #ifndef MEMORY_FIXED_FETCH
    l_prev_iteration_address = mem_address;
    #endif

    // TODO: Notify end of writes

    return return_value;
}

//TODO: remove this probably. Also it should uint16_t
varBool_e bImpWriteFreqRedMap(telemetry_t* i_telemetry)
{
    varBool_e return_value = PCF_TRUE;
    varFor l_num_core = g_SysConfigTable.num_core;

    #ifndef MEMORY_FIXED_FETCH
    //static uint32_t l_iteration_times = 0;
    static uint32_t* l_prev_iteration_address = (uint32_t*)IMP_ADR_OUT_OTHER;
    #endif

    uint32_t* mem_address = NULL;
    #ifdef MEMORY_FIXED_FETCH
	mem_address = (uint32_t*)IMP_ADR_OUT_OTHER;
    #else
    mem_address = l_prev_iteration_address;
    #endif
	size_t address_num_of_bytes = sizeof(uint32_t);
    //TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    //#ifndef MEMORY_FIXED_FETCH
    //mem_address += l_iteration_times * l_num_core * address_num_of_bytes;

    for (varFor i = 0; i < l_num_core; i++)
    {
        *mem_address = (uint32_t)i_telemetry->frequency_reduction_map[i];
        i_telemetry->frequency_reduction_map[i] = BM_RESET;

        mem_address += address_num_of_bytes;

        if (mem_address > (uint32_t*)IMP_ADR_OUT_OTHER_LAST)
        {
            return_value = PCF_FALSE;
            mem_address = (uint32_t*)IMP_ADR_OUT_OTHER;
            if (i < (l_num_core -1))
                i = -1;
        }
    }

    #ifndef MEMORY_FIXED_FETCH
	l_prev_iteration_address = mem_address;
    #endif

    // TODO: Notify end of writes

    return return_value;
}

int lImpUserFunction(void* ptr)
{
    /*
    int* mem_addr = (int*)IMP_ADR_RUN_SIMULATION;
    if (!(*mem_addr))
    {
        bTargetStopTimer();
        vTaskSuspendAll();
        printf("Simulation Stopped\n\r");
        vTargetPcfExit(0);
    }
    */
}

