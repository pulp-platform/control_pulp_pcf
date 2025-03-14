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

/* Libraries Inclusion */
#include "imp_comms.h"

#include "cfg_types.h"
#include "cfg_system.h"
#include "cfg_constants.h"

#include "imp_addresses.h"

#include "tgt_port.h"
#include "tgt_timer.h"

// todo remove
#include "cfg_firmware.h"

/* Others */
// TODO: DELETE:
#include <stdint.h>
#ifdef PRINTF_ACTIVE
#include <stdio.h>
#include "print_float.h"
#endif

// Uncomment to enable Fixed addresses data fetch
// #define MEMORY_FIXED_FETCH

uint32_t l_error_map = BM_RESET;

int intdata_2b_stored[PCF_CORES_MAX] = {0};
float floatdata_2b_stored[PCF_CORES_MAX] = {0};

varBool_e bImpCommsInit(void) {
    varBool_e return_value = PCF_TRUE;
    // TBU

    int *mem_adr = NULL;

    mem_adr = IMP_ADR_RUN_SIMULATION;
    *mem_adr = 1;
    mem_adr = IMP_ADR_PAUSE_SIMULATION;
    *mem_adr = 0;
    mem_adr = IMP_ADR_WORKLOAD_READ;
    *mem_adr = 0;

    return return_value;
}

varBool_e bImpSendCoreFreq(varValue *i_computed_freq) {
    // ATTENTION: this function is called INSIDE an IRQ!
    // KEEP IT SHORT and no STRANGE CALLS.

    varBool_e return_value = PCF_TRUE;
    varFor l_num_cores = g_config_sys.num_cores;

#ifndef MEMORY_FIXED_FETCH
    // static uint32_t l_iteration_times = 0;
    static varValue *l_prev_iteration_address = (varValue *)IMP_ADR_OUT_FIRST_CORE_FREQ;
#endif

    varValue *mem_address = NULL;
#ifdef MEMORY_FIXED_FETCH
    mem_address = (varValue *)IMP_ADR_OUT_FIRST_CORE_FREQ;
#else
    mem_address = l_prev_iteration_address;
#endif
    size_t address_num_of_bytes = sizeof(varValue);
    // TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    // #ifndef MEMORY_FIXED_FETCH
    //  mem_address += l_iteration_times * l_num_cores * address_num_of_bytes;

    for (varFor i = 0; i < l_num_cores; i++) {
        *mem_address = i_computed_freq[i];

        mem_address += address_num_of_bytes;

        if (mem_address > (varValue *)IMP_ADR_OUT_CORE_FREQ_LAST) {
            return_value = PCF_FALSE;
            mem_address = (varValue *)IMP_ADR_OUT_FIRST_CORE_FREQ;
            if (i < (l_num_cores - 1)) i = -1;
        }
    }

#ifndef MEMORY_FIXED_FETCH
    l_prev_iteration_address = mem_address;
#endif

    // TODO: Notify end of writes

    return return_value;
}

varBool_e bImpSendDomainVoltages(varValue *i_computed_voltage) {
    // ATTENTION: this function is called INSIDE an IRQ!
    // KEEP IT SHORT and no STRANGE CALLS.

    varBool_e return_value = PCF_TRUE;
    varFor l_num_domains = g_config_sys.num_domains;

#ifndef MEMORY_FIXED_FETCH
    // static uint32_t l_iteration_times = 0;
    static varValue *l_prev_iteration_address = (varValue *)IMP_ADR_OUT_FIRST_QUAD_VDD;
#endif

    varValue *mem_address = NULL;
#ifdef MEMORY_FIXED_FETCH
    mem_address = (varValue *)IMP_ADR_OUT_FIRST_QUAD_VDD;
#else
    mem_address = l_prev_iteration_address;
#endif
    size_t address_num_of_bytes = sizeof(varValue);
    // TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    // #ifndef MEMORY_FIXED_FETCH
    //  mem_address += l_iteration_times * l_num_domains * address_num_of_bytes;

    for (varFor i = 0; i < l_num_domains; i++) {
        *mem_address = i_computed_voltage[i];

        mem_address += address_num_of_bytes;

        if (mem_address > (varValue *)IMP_ADR_OUT_QUAD_VDD_LAST) {
            return_value = PCF_FALSE;
            mem_address = (varValue *)IMP_ADR_OUT_FIRST_QUAD_VDD;
            if (i < (l_num_domains - 1)) i = -1;
        }
    }

#ifndef MEMORY_FIXED_FETCH
    l_prev_iteration_address = mem_address;
#endif

    // TODO: Notify end of writes

    return return_value;
}

varBool_e bImpReadTempRequest(varValue *o_measured_temp) {
    varBool_e return_value = PCF_TRUE;
    varFor l_num_cores = g_config_sys.num_cores;

#ifndef MEMORY_FIXED_FETCH
    // static uint32_t l_iteration_times = 0;
    static varValue *l_prev_iteration_address = (varValue *)IMP_ADR_IN_FIRST_CORE_TEMP;
#endif

    varValue *mem_address = NULL;
#ifdef MEMORY_FIXED_FETCH
    mem_address = (varValue *)IMP_ADR_IN_FIRST_CORE_TEMP;
#else
    mem_address = l_prev_iteration_address;
#endif
    size_t address_num_of_bytes = sizeof(varValue);
    // TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    // #ifndef MEMORY_FIXED_FETCH
    //  mem_address += l_iteration_times * l_num_cores * address_num_of_bytes;

    for (varFor i = 0; i < l_num_cores; i++) {
        varValue data = *mem_address;
        if (data == 0xCAFECAFE) {
            return_value = PCF_FALSE;
            mem_address = (varValue *)IMP_ADR_IN_FIRST_CORE_TEMP;
            data = *mem_address;
            i = 0;
        }
        o_measured_temp[i] = data;

        mem_address += address_num_of_bytes;

        if (mem_address > (varValue *)IMP_ADR_IN_CORE_TEMP_LAST) {
            return_value = PCF_FALSE;
            mem_address = (varValue *)IMP_ADR_IN_FIRST_CORE_TEMP;
            if (i < (l_num_cores - 1)) i = -1;
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
    varFor l_num_cores = g_config_sys.num_cores;

        varValue* mem_address = TILE_ADR_FIRST_CORE_TEMP_2;

        for (varFor i = 0; i < l_num_cores; i++)
    {
        o_measured_temp[i] = *mem_address;

        mem_address += VAR_VALUE_BYTES_NUM;
    }


    return return_value;
}*/

varBool_e bImpReadInputParameters(struct ctrl_commands *i_table_ptr) {
    varBool_e return_value = PCF_TRUE;
    varFor l_num_cores = g_config_sys.num_cores;

#ifndef MEMORY_FIXED_FETCH
    // static uint32_t l_iteration_times = 0;
    static varValue *l_prev_iteration_address_ft[PCF_CORES_MAX] = {NULL};
    static varValue *l_prev_iteration_address_pwb = (varValue *)IMP_ADR_CMD_POWER_BUDGET;
    static uint32_t *l_prev_iteration_address_bind = (uint32_t *)IMP_ADR_CMD_FIRST_CORE_BINDINGS;

    static int init = 1;

    if (init) {
        for (varFor i = 0; i < l_num_cores; i++)
            l_prev_iteration_address_ft[i] = (varValue *)(IMP_ADR_CMD_FIRST_CORE_FREQ_T +
                                                          4 * 2 * i /* * address_num_of_bytes */);

        init = 0;
    }
#endif

    // TBD: is it better fetch all num_core 1-type-of-data or better for each num_core fetch all
    // data?
    // This is not the most efficient way of doing it, but not important for the date paper
    varValue *mem_address_ft = NULL;
    varValue *mem_address_pwb = NULL;
    uint32_t *mem_address_bind = NULL;

#ifdef MEMORY_FIXED_FETCH
    mem_address_ft = (varValue *)IMP_ADR_CMD_FIRST_CORE_FREQ_T;
    mem_address_pwb = (varValue *)IMP_ADR_CMD_POWER_BUDGET;
    mem_address_bind = (uint32_t *)IMP_ADR_CMD_FIRST_CORE_BINDINGS;
#else
    // mem_address_ft = l_prev_iteration_address_ft[0];
    mem_address_pwb = l_prev_iteration_address_pwb;
    mem_address_bind = l_prev_iteration_address_bind;
    static int l_counter_times_ft[PCF_CORES_MAX] = {0};
    static int l_counter_times_pwb = 0;
    static int l_counter_times_bind = 0;
    varValue *mem_address_times = NULL;
    int times = 0;
#endif

    // I won't fix this ugly thing since it is broken anyway
    size_t address_num_of_bytes = sizeof(varValue);
    // TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    // #ifndef MEMORY_FIXED_FETCH
    //  mem_address += l_iteration_times * l_num_cores * address_num_of_bytes;

    /* Target Frequency */
    for (varFor i = 0; i < l_num_cores; i++) {
#ifndef MEMORY_FIXED_FETCH
        mem_address_ft = l_prev_iteration_address_ft[i];
        // printf("main FT add: %#010x\n\r", mem_address_ft);
        mem_address_times = mem_address_ft + address_num_of_bytes;
        // printf("times FT add: %#010x\n\r", mem_address_times);
        times = (int)*mem_address_times;
        if ((times == -1) || (l_counter_times_ft[i] >= times)) {
            l_counter_times_ft[i] = 0;
            if (times != -1) mem_address_ft += 2 * l_num_cores * address_num_of_bytes;
        } else {
            l_counter_times_ft[i]++;
        }
#endif

        varValue data = *mem_address_ft;
        // if (i==0){printf("imp TF: ");printFloat(data);printf("\n\r");}
        if (data == 0xCAFECAFE) {
            return_value = PCF_FALSE;
            mem_address_ft = (varValue *)IMP_ADR_CMD_FIRST_CORE_FREQ_T;
            data = *mem_address_ft;
            i = 0;
        }

        if (data != i_table_ptr->target_freq[i]) {
            i_table_ptr->target_freq_changed[i] = PCF_TRUE;
            i_table_ptr->target_freq[i] = data;
        } else {
            i_table_ptr->target_freq_changed[i] = PCF_FALSE;
        }

        i_table_ptr->target_freq[i] = data;

#ifdef MEMORY_FIXED_FETCH
        mem_address_ft += address_num_of_bytes;
// mem_address_ft += address_num_of_bytes;
#endif

        if (mem_address_ft > (varValue *)IMP_ADR_CMD_CORE_FREQ_T_LAST) {
            return_value = PCF_FALSE;
            mem_address_ft = (varValue *)IMP_ADR_CMD_FIRST_CORE_FREQ_T;
            if (i < (l_num_cores - 1)) i = -1;
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
    if ((times == -1) || (l_counter_times_pwb > times)) // here strictly
    {
        l_counter_times_pwb = 0;
        if (times != -1) {
            l_prev_iteration_address_pwb += 2 * address_num_of_bytes;
            mem_address_pwb = l_prev_iteration_address_pwb;
        }
    } else {
        l_counter_times_pwb++;
    }
#endif

    varValue datap = *mem_address_pwb;
    if (datap == 0xCAFECAFE) {
        return_value = PCF_FALSE;
        mem_address_pwb = (varValue *)IMP_ADR_CMD_POWER_BUDGET;
        datap = *mem_address_pwb;
    }

    i_table_ptr->total_power_budget = datap;

    if (mem_address_pwb > (varValue *)IMP_ADR_CMD_POWER_BUDGET_LAST) {
        return_value = PCF_FALSE;
        mem_address_pwb = (varValue *)IMP_ADR_CMD_POWER_BUDGET;
    }

/* Bindings */
#ifndef MEMORY_FIXED_FETCH
    mem_address_bind = l_prev_iteration_address_bind;
    mem_address_times = mem_address_bind + address_num_of_bytes;
    times = (int)*mem_address_times;
    if ((times == -1) || (l_counter_times_bind > times)) // here strictly
    {
        l_counter_times_bind = 0;
        if (times != -1) {
            l_prev_iteration_address_bind += 2 * l_num_cores * address_num_of_bytes;
            mem_address_bind = l_prev_iteration_address_bind;
        }
    } else {
        l_counter_times_bind++;
    }
#endif

    for (varFor i = 0; i < l_num_cores; i++) {
        uint32_t data = *mem_address_bind;
        if (data == 0xCAFECAFE) {
            return_value = PCF_FALSE;
            mem_address_bind = (uint32_t *)IMP_ADR_CMD_FIRST_CORE_BINDINGS;
            data = *mem_address_bind;
            i = 0;
        }

        i_table_ptr->core_binding_vector[i] = (varCoreBinding)data;
        mem_address_bind += address_num_of_bytes;
        // mem_address_bind += address_num_of_bytes;

        if (mem_address_bind > (uint32_t *)IMP_ADR_CMD_CORE_BINDINGS_LAST) {
            return_value = PCF_FALSE;
            mem_address_bind = (uint32_t *)IMP_ADR_CMD_FIRST_CORE_BINDINGS;
            if (i < (l_num_cores - 1)) i = -1;
        }
    }

    // TODO: Notify end of writes

    return return_value;
}

varBool_e bImpReadInstructionComposition(struct performance_measures *i_table_ptr) {
    varBool_e return_value = PCF_TRUE;
    varFor l_num_cores = g_config_sys.num_cores;
    varFor l_num_states = g_config_sys.num_wl_states;

    uint32_t *mem_adr = NULL;

#ifndef MEMORY_FIXED_FETCH
// static uint32_t l_iteration_times = 0;
#ifdef USE_INSTRUCTIONS_COMPOSITION
    static uint32_t *l_prev_iteration_address = (uint32_t *)IMP_ADR_IN_FIRST_CORE_INSTR;
#else
    static varValue *l_prev_iteration_address = (varValue *)IMP_ADR_IN_FIRST_CORE_INSTR;
#endif
#endif

// varValue* mem_address = NULL;
#ifdef MEMORY_FIXED_FETCH
#ifdef USE_INSTRUCTIONS_COMPOSITION
    uint32_t *mem_address = (uint32_t *)IMP_ADR_IN_FIRST_CORE_INSTR;
#else
    varValue *mem_address = (varValue *)IMP_ADR_IN_FIRST_CORE_INSTR;
#endif
#else
    // TODO, here it is not separated between USE_INSTRUCTIONS_COMPOSITION and not
    varValue *mem_address = l_prev_iteration_address;
#endif
    size_t address_num_of_bytes = sizeof(varValue);
    // TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    // #ifndef MEMORY_FIXED_FETCH
    //  mem_address += l_iteration_times * l_num_cores * address_num_of_bytes;

    for (varFor i = 0; i < l_num_cores; i++) {
#ifdef USE_INSTRUCTIONS_COMPOSITION
        for (varFor state = 0; state < l_num_states; state++) {
            uint32_t data = *mem_address;
            if (data == 0xCAFECAFE) {
                return_value = PCF_FALSE;
                mem_address = (uint32_t *)IMP_ADR_IN_FIRST_CORE_INSTR;
                data = *mem_address;
                i = 0;
                state = 0;
            }

            i_table_ptr->perc[i][state] = (uint8_t)data;
            // printf("test workload: %d\n\r", (int)i_table_ptr->perc[i][state]);

            mem_address += address_num_of_bytes;

            if (mem_address > (uint32_t *)IMP_ADR_IN_CORE_INSTR_LAST) {
                return_value = PCF_FALSE;
                mem_address = (uint32_t *)IMP_ADR_IN_FIRST_CORE_INSTR;
                if (i < (l_num_cores - 1)) {
                    i = -1;
                    state = -1;
                }
            }
        }
#else // SINGLE Ceff
        varValue data = *mem_address;
        if (data == 0xCAFECAFE) {
            return_value = PCF_FALSE;
            mem_address = (varValue *)IMP_ADR_IN_FIRST_CORE_INSTR;
            data = *mem_address;
            i = 0;
        }

        i_table_ptr->ceff[i] = data;

        mem_address += address_num_of_bytes * l_num_states;
        // TODO: test:
        // printf ("mem_add = %x\n\r", mem_address);

        if (mem_address > (varValue *)IMP_ADR_IN_CORE_INSTR_LAST) {
            return_value = PCF_FALSE;
            mem_address = (varValue *)IMP_ADR_IN_FIRST_CORE_INSTR;
            if (i < (l_num_cores - 1)) i = -1;
        }
#endif
    }

#ifndef MEMORY_FIXED_FETCH
    l_prev_iteration_address = mem_address;
#endif

    mem_adr = IMP_ADR_WORKLOAD_READ;
    *mem_adr = 1;

    // TODO: Notify end of writes

    return return_value;
}

varBool_e bImpReadPowerMeasure(struct power_measures *i_table_ptr) {
    varBool_e return_value = PCF_TRUE;
    varFor l_num_cores = g_config_sys.num_cores;

#ifndef MEMORY_FIXED_FETCH
    // static uint32_t l_iteration_times = 0;
    static varValue *l_prev_iteration_address = (varValue *)IMP_ADR_IN_POWER_CPU;
#endif

    varValue *mem_address = NULL;
#ifdef MEMORY_FIXED_FETCH
    mem_address = (varValue *)IMP_ADR_IN_POWER_CPU;
#else
    mem_address = l_prev_iteration_address;
#endif
    size_t address_num_of_bytes = sizeof(varValue);
    // TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    // #ifndef MEMORY_FIXED_FETCH
    //  mem_address += l_iteration_times * l_num_cores * address_num_of_bytes;

    varValue data = *mem_address;
    if (data == 0xCAFECAFE) {
        return_value = PCF_FALSE;
        mem_address = (varValue *)IMP_ADR_IN_POWER_CPU;
        data = *mem_address;
    }
    i_table_ptr->total = data;

    mem_address += address_num_of_bytes;

    if (mem_address > (varValue *)IMP_ADR_IN_POWER_CPU_LAST) {
        return_value = PCF_FALSE;
        mem_address = (varValue *)IMP_ADR_IN_POWER_CPU;
    }

#ifndef MEMORY_FIXED_FETCH
    l_prev_iteration_address = mem_address;
#endif

    // TODO: Notify end of writes

    return return_value;
}

// TODO: remove this probably. Also it should uint16_t
varBool_e bImpWriteFreqRedMap(telemetry_t *i_telemetry) {
    varBool_e return_value = PCF_TRUE;
    varFor l_num_cores = g_config_sys.num_cores;

#ifndef MEMORY_FIXED_FETCH
    // static uint32_t l_iteration_times = 0;
    static uint32_t *l_prev_iteration_address = (uint32_t *)IMP_ADR_OUT_FIRST_FREQREDMAP;
#endif

    uint32_t *mem_address = NULL;
#ifdef MEMORY_FIXED_FETCH
    mem_address = (uint32_t *)IMP_ADR_OUT_FIRST_FREQREDMAP;
#else
    mem_address = l_prev_iteration_address;
#endif
    size_t address_num_of_bytes = sizeof(uint32_t);
    // TODO: Compiler doesn't work properly: it multiply everything for 4.
    address_num_of_bytes = 1;

    // #ifndef MEMORY_FIXED_FETCH
    //  mem_address += l_iteration_times * l_num_cores * address_num_of_bytes;

    for (varFor i = 0; i < l_num_cores; i++) {
        *mem_address = (uint32_t)i_telemetry->frequency_reduction_map[i];
        i_telemetry->frequency_reduction_map[i] = BM_RESET;

        mem_address += address_num_of_bytes;

        // added //TODO REMOVE
        float *mem_floatt = mem_address; //+ address_num_of_bytes*l_num_cores;
        *mem_floatt = (float)i_telemetry->avg_core[i].est_pw;
        mem_address += address_num_of_bytes;
        // endadded

        if (mem_address > (uint32_t *)IMP_ADR_OUT_FREQREDMAP_LAST) {
            return_value = PCF_FALSE;
            mem_address = (uint32_t *)IMP_ADR_OUT_FIRST_FREQREDMAP;
            if (i < (l_num_cores - 1)) i = -1;
        }
    }

#ifndef MEMORY_FIXED_FETCH
    l_prev_iteration_address = mem_address;
#endif

    // TODO: Notify end of writes

    return return_value;
}

int lImpUserFunction(void *ptr) {

    int *mem_addr = (int *)IMP_ADR_RUN_SIMULATION;
    if (!(*mem_addr)) {
        bTargetStopTimer();
        vTaskSuspendAll();
        printf("Simulation Stopped\n\r");
        vTargetPcfExit(0);
    }
}
