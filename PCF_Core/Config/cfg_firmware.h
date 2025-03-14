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

#ifndef _PCF_FIRMWARE_CONFIG_H_
#define _PCF_FIRMWARE_CONFIG_H_

/* FreeRTOS Inclusions. */
#include "FreeRTOS.h"
#include "task.h"

/* Libraries Inclusion */
#include "cfg_types.h"
#include "cfg_system.h"
#include "cfg_control.h"

/** Definitions **/
/* these has to be manually changed */
/** Tasks **/
// Max Number of Tasks
#define MAX_NUM_TASKS 7 //(6 tasks+1debug+1idle)
#define NUM_PERIODIC_TASKS 6

/****** Private ******/
/* NOT CONFIGURABLE */
/** Code Configuration **/
// #define LMS_ACTIVE
// #define DEBUG_ACTIVE
// #define MEASURE_ACTIVE						2 // 1: N measurements, 2:
//  Validation
// #define MEASURE_N_ITERATION 				8 // This is needed for both debug and
//  Measure
// #define MEASURE_N_OUTPUTS					10 // This is the number of
// outputted
//  measures
// #define GPIO_TEST
// #define CI_TEST
// #define CI_TEST_ITERATION					8
#define JO_TBR // I Include in this stuff that must be removed once the GAP8 is overcome. I started
               // using this 6/7/20 so some stuff before this date
// may not be included in this

// if defined, when the tap is not exact, it will be increased instead of decreased
#define TAP_INTERVAL_APPROXIMATE_CEIL

/****** Private END ******/

/******************************/
/** Firmware Configuration ****/
/******************************/
struct tasks_config_table {

    /**** Tasks Configuration *****/
    uint32_t tap_period_us; // Time in us //when change this also hange TELEMETRY_POLLING_FREQUENCY
    // following values will be floored down to a multiple of the tap period.
    uint32_t tasks_period_us[MAX_NUM_TASKS];      // period in us. 0 for non-periodic tasks
    uint32_t tasks_tap_multiplier[MAX_NUM_TASKS]; // internal
    uint32_t tasks_priority[MAX_NUM_TASKS];       // task priority (higher number = more important)
};

struct code_config_table {

    /***** Code configuration *****/
    varBool_e use_watchdogs;
    varBool_e use_secure_io;
    varBool_e use_tests_on_numbers; // TODO: configASSERT seem not to work

    /*** Telemetry Configuration **/
    varBool_e use_error_map;
};

typedef struct _task_param {
    uint32_t error_map;
    uint32_t task_id;
    // #ifdef PCF_USE_CLUSTER
    void *cl_ptr;
    uint32_t num_cl_cores;
    // #endif
} task_param_t;

/******************************/
/***** Global Variables *******/
/******************************/
struct tasks_config_table g_TaskConfigTable;
struct code_config_table g_CodeConfigTable;

/* Utilities to control tasks */
extern TaskHandle_t taskHandles[MAX_NUM_TASKS];
// extern int16_t g_tasks_mul_factor[MAX_NUM_TASKS]; //signed because counters are signed

typedef struct _task_shared_table {
    struct ctrl_output op;
} shared_task_t;

extern shared_task_t g_shared_task;

#endif // lib #ifdef
