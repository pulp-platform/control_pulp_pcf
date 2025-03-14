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

#ifndef _PCF_SYS_CONFIG_H_
#define _PCF_SYS_CONFIG_H_

/* Libraries Inclusion */
#include "cfg_types.h"

/** Definitions **/
/* these has to be manually changed */
#define PCF_CORES_MAX 36
#define PCF_DOMAINS_MAX 4 // 36 //4

#define PCF_WL_STATES 5
#define PCF_MODEL_COEFF 2

#define PCF_VOLTAGE_LEVELS 15

/******************************/
/* Configuration of Processor */
/******************************/
// TODO: Here probably I should modify this:
/*
 * critical_temperature, idle/max power and freq, etc. and also init values should be
 * 	put in a structure, so that I can cast Core, Ram, etc.
 * Also, since multiple chiplets and sockets, I should create an array of num_*
 * Also since heterogeneous cores, etc. I should create an array of the structures
 */
typedef struct _config_sys_table {

    uint16_t num_cores;    // Number of Cores of each chiplet
    uint16_t num_domains;  // Number of Power Domains
    uint16_t num_chiplets; // Number of Chiplet per each Processor
    uint16_t num_sockets;  // Number of Processor/Socket per Motherboard

    uint16_t num_voltage_levels;
    varValue voltage_table[PCF_VOLTAGE_LEVELS * 3]; //*3 because: 0->voltage level, 1->Fmin, 2->Fmax
    uint16_t core_domain_id[PCF_CORES_MAX];
    uint16_t num_cores_per_domain[PCF_DOMAINS_MAX];

    /* Parameters of the Cores */
    uint16_t num_wl_states;   // Number of states
    uint16_t num_model_coeff; // Number of power model coefficients

    varValue core_critical_temperature; // Max Temperature for each Core in Kelvin
    varValue core_idle_power;           // Min Power for a Single Core to stay alive
    varValue core_max_power_single;     // Max Power for each Core
    varValue core_min_frequency;
    varValue core_max_frequency;
    varValue base_power_coeff[PCF_WL_STATES][PCF_MODEL_COEFF];

    /* Initialization Values */
    varValue init_core_freq;
    varValue init_core_volt;
    varValue init_core_ceff;

} config_sys_t;

/******************************/
/***** Global Variables *******/
/******************************/
extern config_sys_t g_config_sys;

#endif // lib #ifdef
