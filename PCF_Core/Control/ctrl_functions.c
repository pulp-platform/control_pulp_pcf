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

/* Libraries Inclusion */
#include "ctrl_functions.h"

#include "cfg_types.h"
#include "cfg_system.h"
#include "cfg_control.h"
#include "cfg_constants.h"
#include "cfg_firmware.h"

#include "ctrl_math.h"

// TOCHINK?
#include "tgt_cluster.h"

// TODO
#include "pcf_taskFunctions.h"
#include "sec_functions.h"

// TODO: remove!!!
#include "imp_comms.h"
#include "imp_addresses.h"
// REMOOOOOOOOOOVEEE!!!
#include "timer.h"
#include "timer_hal.h"
#include "timer_irq.h"
#include "cluster/event_unit.h"

// Others
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

/*** Internal Functions ***/
varValue fCheckFrequencies(varFor i_start, varFor i_end, varFor i_increment,
                           struct ctrl_task_index *tptr, void **args);
varValue fMultiInputProcess(varFor i_start, varFor i_end, varFor i_increment,
                            struct ctrl_task_index *tptr, void **args);
inline void vPowerBudgetProcess(struct ctrl_task_index *tptr);
inline void vPowerConsumptionAdaptation(struct ctrl_task_index *tptr);
varValue fComputeEstimatedTotalPower(varFor i_start, varFor i_end, varFor i_increment,
                                     struct ctrl_task_index *tptr, void **args);
varValue fCorePowerComputation(varFor i_start, varFor i_end, varFor i_increment,
                               struct ctrl_task_index *tptr, void **args);
varValue fVoltageCastVote(varFor i_start, varFor i_end, varFor i_increment,
                          struct ctrl_task_index *tptr, void **args);
varValue fVoltageChoice(varFor i_start, varFor i_end, varFor i_increment,
                        struct ctrl_task_index *tptr, void **args);
varValue fFVpreAdaptation(varFor i_start, varFor i_end, varFor i_increment,
                          struct ctrl_task_index *tptr, void **args);
inline void vFVdomainAdaptation(struct ctrl_task_index *tptr);
varValue fDummyPowerReduction(varFor i_start, varFor i_end, varFor i_increment,
                              struct ctrl_task_index *tptr, void **args);
inline void vAlphaPowerReduction(varValue i_delta_power, varValue est_total_pw,
                                 struct ctrl_task_index *tptr);
varValue fComputeAlpha(varFor i_start, varFor i_end, varFor i_increment,
                       struct ctrl_task_index *tptr, void **args);
inline void vComputeAlphaBindings(struct ctrl_task_index *tptr);
varValue fNormalizeApplyAlpha(varFor i_start, varFor i_end, varFor i_increment,
                              struct ctrl_task_index *tptr, void **args);
// varValue fAlphaDomain(varFor i_start, varFor i_end, varFor i_increment, struct ctrl_task_index*
// tptr, void** args);
varValue fComputeTemperatureReduction(varFor i_start, varFor i_end, varFor i_increment,
                                      struct ctrl_task_index *tptr, void **args);
inline void vComputeFrequency(struct ctrl_task_index *tptr);
varValue fFrequencyCouplingTransformation(varFor i_start, varFor i_end, varFor i_increment,
                                          struct ctrl_task_index *tptr, void **args);
// This function has the same structure, but it is not called with the macro
varFor lPopulateCouplingStructure(varFor i_start, varFor i_end, varFor i_increment,
                                  struct ctrl_task_index *tptr, void **args);
// no inline is ok!
void vProcessFrequency(varFor i_core, varValue i_frequency, struct ctrl_task_index *tptr);
varValue fFrequency2PowerConversion(varFor i_start, varFor i_end, varFor i_increment,
                                    struct ctrl_task_index *tptr, void **args);

#define CP_CLUSTER_OVERHEAD 200
#define MAX_NUM_CLUSTER_CORE 8

// TODO REMOVE:
int abrakazam = 0;

void vDummyCluster(void *args) {
// TODO: Do it better!
#ifndef PCF_USE_CLUSTER_PARALL
    // if (abrakazam % 3 == 2)
    //	vMainTaskControlAlgorithm(args, 8);
    // else
    vMainTaskControlAlgorithm(args);
#else
    vMainTaskControlAlgorithm(args);
#endif
}
void MACRO_CP_CLUSTER_PARALLELIZE(struct function_param *param) {
    int exec_gain = param->exec_time / (int)param->parall_num + CP_CLUSTER_OVERHEAD;
    if ((param->parall_num > 1) && (exec_gain < param->exec_time)) {
        printf("PLplPlPl\n\r");
        if (param->return_value == NULL) {
            bTargetClusterFork(vDummyFork, (void *)param, param->parall_num);
        } else {
            varValue accum[MAX_NUM_CLUSTER_CORE] = {0};
            varValue *ptr_hold = param->return_value;
            param->return_value = (void *)accum;
            bTargetClusterFork(vDummyFork, (void *)param, param->parall_num);
            *ptr_hold = VD_ZERO;
            for (varFor ccore = 0; ccore < param->parall_num; ccore++) {
                *ptr_hold += accum[ccore];
            }
        }
    } else {
        if (param->return_value == NULL) {
            param->f(0, param->total_iterations, 1, param->igl, param->ptr_args);
        } else {
            *(param->return_value) =
                param->f(0, param->total_iterations, 1, param->igl, param->ptr_args);
        };
    }
}

// TODO:
#define PW_CHANGED_ALPHA_COUNTER 36
#define TF_CHANGED_ALPHA_COUNTER 16

// TODO
void vMainTaskControlInit(task_param_t *tp) {
}

// TODO change name and move position
void vConfigValueCtrlInit(ctrl_values_table_t *values) {
    // Initialization:
    // tp->error_map = BM_RESET;
    // tp->task_id = 5397;
    // tp->values = &c_values_table;
    // tp->tel = &c_telemetry_table;

    values->telemetry_counter = 5; // TODO
    values->ctrl_counter = 0;
    for (varFor core = 0; core < g_config_sys.num_cores; core++) {
        values->target_core_power[core] = g_config_sys.core_idle_power;
        values->freq_red_map[core] = BM_RESET;
    }

    varValue data = 0;
    for (varFor state = 0; state < g_config_sys.num_wl_states; state++) {
        data += 1.0f / g_config_sys.num_wl_states * g_config_sys.base_power_coeff[state][1];
        //(ctrl_measures).performance_measures.ceff = ... ;
    }
    for (varFor core = 0; core < g_config_sys.num_cores; core++) {
        values->ip.workload[core] = data;
        values->ip.temp_der[core] = 0;
        values->ip.prev_temp[core] = 25.0f; // TODO //TBD
    }

    values->ip.prev_power_budget = g_config_sys.num_cores * 2.0; // TODOOOO!!
    values->ip.power_budget_changed = PCF_FALSE;
    values->ip.total_power_adaptation_term = 0;
    values->ip.prev_tot_power[0] = g_config_sys.num_cores * 2.0; // TODOOOO!!
    values->ip.prev_tot_power[1] = g_config_sys.num_cores * 2.0; // TODOOOO!!

    // values->ma.alpha = p_config_ctrl->vv_moving_average_alpha;
    values->ma.alpha_counter = PW_CHANGED_ALPHA_COUNTER; // init this to be flexible
    for (varFor core = 0; core < g_config_sys.num_cores; core++) {
        values->ma.og_freq[core] = g_config_sys.init_core_freq;
        values->ma.freq_point[core] = g_config_sys.init_core_freq;
    }

    for (varFor core = 0; core < g_config_sys.num_cores; core++) {
        values->th.ctrl_cmd[core] = VD_ZERO;
        values->th.hyst_thresh_reached[core] = PCF_FALSE;
        // values->measured_temperature[core]	= VD_TEMP_INIT;
        values->th.fuzzy_vcred[core] = 0;
    }

    // TODO:
    for (varFor core = 0; core < g_config_sys.num_cores; core++) {
        values->op.computed_core_frequency[core] = g_config_sys.init_core_freq;
        values->target_core_power[core] = 1.0; // TODO
    }
    for (varFor domain = 0; domain < g_config_sys.num_domains; domain++) {
        values->op.store_max_freq[domain] = g_config_sys.voltage_table[(0) * 3 + 2];
        values->op.computed_domain_voltage[domain] = g_config_sys.voltage_table[(0) * 3 + 0];
    }

    /*
    for (varFor i = 0; i < g_config_sys.num_cores; i++)
    {
            c_telemetry_table.avg_core[i].est_pw 		= g_config_sys.core_idle_power;

            c_telemetry_table.avg_core[i].freq	= g_config_sys.init_core_freq;
            c_telemetry_table.avg_core[i].voltage 	= g_config_sys.init_core_volt;
            c_telemetry_table.avg_core[i].temp = VD_TEMP_INIT;
            c_telemetry_table.frequency_reduction_map[i]			= BM_RESET;
            //
    }
    for (varFor i = 0; i < g_config_sys.num_domains; i++)
    {
            c_telemetry_table.avg_domain[i].est_pw		= 0;
    }
    c_telemetry_table.chip_est_pw				= 0;
    */
    // c_telemetry_table.chip_avg_measured_power 				= 0;
    // c_telemetry_table.power_budget_exceed_us				= 0;

    // if (bWriteGlobalVariable(&c_values_table, G_CTRL_PARAMETER_TABLE, GLOBAL_WRITE_ALL,
    // sizeof(c_values_table), c_task_id) != PCF_TRUE)
    //{
    // TODO

    //	#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //	printf("bWriteGlobalVariable issue: l_ctrl_inputs_table, id: %d\n\r", c_task_id);
    //	#endif
    //}

    // if (bWriteGlobalVariable(&c_telemetry_table, G_TELEMETRY, GLOBAL_WRITE_ALL,
    // sizeof(c_telemetry_table), c_task_id) != PCF_TRUE)
    //{
    // TODO

    //	#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //	printf("bWriteGlobalVariable issue: c_telemetry_table, id: %d\n\r", c_task_id);
    //	#endif
    //}
}

// TODO:
// static ctrl_config_table_t c_parameters_table;
// static ctrl_inputs_t c_inputs_table;

void vMainTaskControlAlgorithm(task_param_t *tp) {

    /*-----------------------------------------------*/
    /****************** Control Vars *****************/
    /*-----------------------------------------------*/
    struct ctrl_task_index l_cti;
    // TBC: is it risky to play with all these pointers?
    varFor parallel_num = (varFor)tp->num_cl_cores;
    ctrl_values_table_t *p_values_ctrl = tp->cl_ptr;
    ctrl_inputs_t *p_input = &g_input_ctrl;
    ctrl_config_table_t *p_config_ctrl;
    config_sys_t *p_config_sys = &g_config_sys;
#ifdef PCF_USE_CLUSTER
    ctrl_values_table_t l_values_ctrl; // TBC: I cannot avoid the case "I have the Cluster, but
                                       // running in the FC"
    ctrl_inputs_t l_input; // TBC: I cannot avoid the case "I have the Cluster, but running in the
                           // FC"
    // TODO: REMOVE
    ctrl_config_table_t cl_config_ctrl;
    config_sys_t cl_config_sys;
#endif

/*-----------------------------------------------*/
/**************** 0: Copying Data ****************/
/*-----------------------------------------------*/

// TODO: add a check: compare execution between copying data, and not copying data.
#ifdef PCF_USE_CLUSTER
#if (MEASURE_ACTIVE == 1)
    // Setup FC timer 0, LO
    timer_id_t timer0_id = TIMER_LO_ID;
    reset_timer_cl(timer0_id);
    start_timer_cl(timer0_id);
    Timer_Data_t timerBuffer_c[8] = {{'0', 0}};
    varFor Timerindex_c = 0;
#endif

    if (parallel_num > 0) // If we are on the cluster //TBC: if there exist a better function
    {
        /** DMA **/
        unsigned int dma_id_value =
            pulp_cl_idma_memcpy(&l_values_ctrl, tp->cl_ptr, sizeof(ctrl_values_table_t));

        unsigned int dma_id_input =
            pulp_cl_idma_memcpy(&l_input, &g_input_ctrl, sizeof(ctrl_inputs_t));

        // TODO: there must be a better way to do this.
        //	A) These values are almost constant, so I need to create a "global cluster variable"
        //	B) If I want to change these values, I create a function to do so, and this function
        // will also copy the changed
        //		values in the cluster
        // p_config_ctrl = &g_ControlConfigTable; //TODO: &gc_ControlConfigTable;
        // p_config_sys = &g_config_sys; //TODO: &gc_config_sys;
        unsigned int remove_cfg_ctrl = pulp_cl_idma_memcpy(&cl_config_ctrl, &g_ControlConfigTable,
                                                           sizeof(ctrl_config_table_t));

        unsigned int remove_cfg_sys =
            pulp_cl_idma_memcpy(&cl_config_sys, &g_config_sys, sizeof(config_sys_t));

        // TBD: here dma wasted?
        plp_cl_dma_wait(dma_id_value);
        p_values_ctrl = &l_values_ctrl;

        // TBD: here dma wasted?
        plp_cl_dma_wait(dma_id_input);
        p_input = &l_input;

        // TOREMOVE
        plp_cl_dma_wait(remove_cfg_ctrl);
        p_config_ctrl = &cl_config_ctrl;
        plp_cl_dma_wait(remove_cfg_sys);
        p_config_sys = &cl_config_sys;

        // printf("clust: tf [%d]: ", l_input.cmd.target_freq_changed[0]);
        // printFloat(l_input.cmd.target_freq[0]);
        // printf("\n\r");
    } else {
        p_config_ctrl = &g_ControlConfigTable;
        p_config_sys = &g_config_sys; // TODO: &gc_config_sys;
    }
#else
    p_config_ctrl = &g_ControlConfigTable;
    p_config_sys = &g_config_sys; // TODO: &gc_config_sys;
#endif

    /*-----------------------------------------------*/
    /************** 0.1: Populate Struct *************/
    /*-----------------------------------------------*/

    l_cti.config_ctrl = p_config_ctrl;
    l_cti.config_sys = p_config_sys;
    l_cti.values = p_values_ctrl;
    l_cti.inputs = p_input;
    l_cti.tel = NULL; //&c_telemetry_table;
    l_cti.tp = tp;

    struct function_param p;

    void *l_ptr_args[3]; // TODO becareful when choosing this number.
    p.parall_num = parallel_num;
    p.igl = &l_cti;

    l_cti.fp = &p;

    /*
    printf("tf [%d]: ", p_input->cmd.target_freq_changed[0]);
    printFloat(p_input->cmd.target_freq[0]);
    printf(" - wrkl: ");
    printFloat(p_values_ctrl->ip.workload[0]);
    printf("\n\r pidinta: %d - numcore: %d/%d - ", p_config_ctrl->conf.use_pid_integral_action,
            p_config_sys->num_cores, p_config_sys->num_domains );
    printFloat(p_config_sys->base_power_coeff[0][0]);

    printf("\n\r");
    */
    /*
    printf("ctrl TF: ");
printFloat(p_input->cmd.target_freq[0]);
printf("\n\r");
    */

    if (p_values_ctrl->ctrl_counter >=
        10 * p_config_ctrl->therm.fuzzy_mul * p_config_ctrl->therm.der_mul) // TODO: mcm, and 10
    {
        p_values_ctrl->ctrl_counter =
            2 * p_config_ctrl->therm.fuzzy_mul * p_config_ctrl->therm.der_mul;
    }
    p_values_ctrl->ctrl_counter++;
    // printf("ctrlc: %d\n\r", p_values_ctrl->ctrl_counter);

    /*-----------------------------------------------*/
    /********* 1: Write Computed Values(k-1) *********/
    /*-----------------------------------------------*/

#ifdef CTRL_DEBUG
    printf("F0: ");
    printFloat(p_values_ctrl->op.computed_core_frequency[0]);
    printf("\n\r");
#endif

    // Check Frequencies:
    p.f = fCheckFrequencies;
    p.total_iterations = p_config_sys->num_cores;
    p.exec_time = 1;
    p.ptr_args = NULL;
    p.return_value = NULL;
    MACRO_CP_CLUSTER_PARALLELIZE(&p);

    // send frequencies
    if (bSecureSendCoreFrequencies(p_values_ctrl->op.computed_core_frequency) != PCF_TRUE) {
        // TODO

#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
        printf("Error in sending Frequencies\n\r");
#endif
    }

#if (MEASURE_ACTIVE == 1)
#ifdef PCF_USE_CLUSTER
    timerBuffer_c[Timerindex_c++] = (Timer_Data_t){'s', get_timer_cnt_cl(timer0_id)};
#else
    timerBuffer[Timerindex++] = (Timer_Data_t){'s', lMeasureReadCsr(MEASURE_ZEROING)};
#endif
#endif

    /*-----------------------------------------------*/
    /*************** 2: Read Values(k) ***************/
    /*-----------------------------------------------*/

    if (bSecureReadCoreTemperatures(p_input->msr.temp.core) != PCF_TRUE) {
        // TODO

#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
        printf("Error in reading Temperatures\n\r");
#endif
    }

    // printf("Temp: ");
    // printFloat(p_input->msr.temp.core[0]);
    // printf("\n\r");

#if (MEASURE_ACTIVE == 1)
#ifdef PCF_USE_CLUSTER
    timerBuffer_c[Timerindex_c++] = (Timer_Data_t){'r', get_timer_cnt_cl(timer0_id)};
#else
    timerBuffer[Timerindex++] = (Timer_Data_t){'r', lMeasureReadCsr(MEASURE_ZEROING)};
#endif
#endif

    /*-----------------------------------------------*/
    /**************** 3: Read DATA(k) ****************/
    /*-----------------------------------------------*/

    // printf("Tf1: ");
    // printFloat(p_input->target_freq[0]);
    // printf("\n\r");

    /*-----------------------------------------------*/
    /************** 4: DATA(k) analysis **************/
    /*-----------------------------------------------*/

    p.f = fMultiInputProcess;
    p.total_iterations = p_config_sys->num_cores;
    p.exec_time = 1;
    p.ptr_args = NULL;
    p.return_value = NULL;
    MACRO_CP_CLUSTER_PARALLELIZE(&p);

    /* Check if power budget has changed */
    vPowerBudgetProcess(&l_cti);

    /* Power Model Estimation */
    vPowerConsumptionAdaptation(&l_cti);

    /*-----------------------------------------------*/
    /************ 5: Compute Power Model *************/
    /*-----------------------------------------------*/

    /* Choosing the Voltage */
    // TODO can I unite the two parts? while I casting votes I increase the accumulator, and I stop
    // casting votes when I reached the value?
    // actually not, I cannot

    // TBD: question, is it better to put the appropriate voltage level or still cast vote for the
    // domain?
    // probably casting votes.
    // Rememeber!! This part has to be REAL too, because total power may be < budget and so, don't
    // go under alpha red
    // still NR will apply
    // but based on a wrong Pc (if not done correctly), so it will be sub-optimal

    // Casting Vote
    uint16_t voltage_voting[PCF_VOLTAGE_LEVELS * PCF_DOMAINS_MAX] = {0}; // TBC

    // Casting votes to populate voltage array level
    // TODO: to improve parallelization I could divide by domain and add a if domain == id_domain.
    p.f = fVoltageCastVote;
    p.total_iterations = p_config_sys->num_cores;
    p.exec_time = 1;
    l_ptr_args[0] = (void *)&voltage_voting;
    p.ptr_args = l_ptr_args;
    p.return_value = NULL;
    MACRO_CP_CLUSTER_PARALLELIZE(&p);

    /*
    if (p_values_ctrl->ctrl_counter % p_config_ctrl->therm.fuzzy_mul == 0)
    for (varFor i=0; i < PCF_VOLTAGE_LEVELS*PCF_DOMAINS_MAX; i++)
    {
            if (i%PCF_VOLTAGE_LEVELS==0)
            {printf("\n\r");}
            printf("%d - ", voltage_voting[i]);
    }
    printf("\n\r");
    */

    // Deciding the Voltage
    p.f = fVoltageChoice;
    p.total_iterations = p_config_sys->num_domains;
    p.exec_time = 1;
    p.ptr_args = l_ptr_args;
    p.return_value = NULL;
    MACRO_CP_CLUSTER_PARALLELIZE(&p);

#ifdef CTRL_DEBUG
    printf("V1: ");
    printFloat(p_values_ctrl->op.computed_domain_voltage[0]);
    printf("\n\r");
#endif

#if (MEASURE_ACTIVE == 1)
#ifdef PCF_USE_CLUSTER
    timerBuffer_c[Timerindex_c++] = (Timer_Data_t){'s', get_timer_cnt_cl(timer0_id)};
#else
    timerBuffer[Timerindex++] = (Timer_Data_t){'i', lMeasureReadCsr(MEASURE_ZEROING)};
#endif
#endif

/*** Pre-Voltage Pre-Freq Adaptation ***/
#ifdef USE_CTRL_FUZZY
    if (p_values_ctrl->ctrl_counter % p_config_ctrl->therm.fuzzy_mul == 0) {
        // varValue fuzzy_core_table[PCF_CORES_MAX];

        p.f = fFVpreAdaptation;
        p.total_iterations = p_config_sys->num_cores;
        p.exec_time = 1;
        // l_ptr_args[0] = (void*)fuzzy_core_table;
        p.ptr_args = NULL;
        p.return_value = NULL;
        MACRO_CP_CLUSTER_PARALLELIZE(&p);
    }

    // TODO: can I optimize or parallelize it?
    vFVdomainAdaptation(&l_cti);

#if (MEASURE_ACTIVE == 1)
#ifdef PCF_USE_CLUSTER
    timerBuffer_c[Timerindex_c++] = (Timer_Data_t){'T', get_timer_cnt_cl(timer0_id)};
#else
    timerBuffer[Timerindex++] = (Timer_Data_t){'T', lMeasureReadCsr(MEASURE_ZEROING)};
#endif
#endif

#endif // USE_CTRL_FUZZY

    /*** Compute Target Power per Core ***/
    varValue estimated_total_power;
    l_ptr_args[0] = (void *)p_values_ctrl->core_frequency;
    p.f = fCorePowerComputation;
    p.total_iterations = p_config_sys->num_cores;
    p.exec_time = 1;
    p.ptr_args = l_ptr_args;
    p.return_value = &estimated_total_power;
    MACRO_CP_CLUSTER_PARALLELIZE(&p);

    // test_on_number
    if (estimated_total_power <= 0) {
// TODO c_error_map |= BM_ERROR_NUMBERS_VALUE;
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
        // printf("Number issue, estimated_t otal_power: %f\n\r", estimated_total_power);
        printf("Number issue, estimated_total_power: ");
        printFloat(estimated_total_power);
        printf("\n\r");
#endif

        // TBD:
        estimated_total_power = p_config_sys->core_idle_power * (varValue)p_config_sys->num_cores;
        for (varFor i = 0; i < p_config_sys->num_cores; i++) {
            p_values_ctrl->target_core_power[i] = p_config_sys->core_idle_power;
        }
    }

#ifdef CTRL_DEBUG
    printf("P1: ");
    printFloat(p_values_ctrl->target_core_power[0]);
    printf("\n\r");
#endif

#ifdef USE_CTRL_STRCT_INVERSE
    // reduce Temperature:
    p.f = fComputeTemperatureReduction;
    p.total_iterations = p_config_sys->num_cores;
    p.exec_time = 1 p.ptr_args = NULL;
    p.return_value = NULL;
    MACRO_CP_CLUSTER_PARALLELIZE(&p);
    estimated_total_power = 0;
    for (varFor i = 0; i < p_config_sys->num_cores; i += 1) {
        estimated_total_power += p_values_ctrl->target_core_power[i];
    }
#endif // USE_CTRL_STRCT_INVERSE

#ifdef USE_CTRL_STRCT_VB
    varValue reduced_core_power[PCF_CORES_MAX];
    varValue og_core_power[PCF_CORES_MAX];
    for (varFor i = 0; i < p_config_sys->num_cores; i++) {
        og_core_power[i] = p_values_ctrl->target_core_power[i];
        // reduced_core_power[i] = p_values_ctrl->taget_core_power[i];
    }
#endif

    /*** Compute Delta Power ***/
    // TODO: domains budgets
    varValue delta_power = estimated_total_power - p_input->cmd.total_power_budget +
                           p_values_ctrl->ip.total_power_adaptation_term;

    /*
    printf("Total / delta: ");
    printFloat(p_input->cmd.total_power_budget);
    printf(" / ");
    printFloat(delta_power);
    printf("\n\r");
    */

    /*-----------------------------------------------*/
    /**************** 6: Reduce Power ****************/
    /*-----------------------------------------------*/

    if (delta_power > 0) {
        vAlphaPowerReduction(delta_power, estimated_total_power, &l_cti);
        /*
        p.f = fDummyPowerReduction;
        p.total_iterations = p_config_sys->num_cores;
        p.exec_time = 5000;
        varValue hold = (estimated_total_power);
        l_ptr_args[0] = (void*)&delta_power;
        l_ptr_args[1] = (void*)&hold;
        p.ptr_args = l_ptr_args;
        p.return_value = NULL;
        MACRO_CP_CLUSTER_PARALLELIZE(&p);
        */
    }

#ifdef CTRL_DEBUG
    printf("P2: ");
    printFloat(p_values_ctrl->target_core_power[0]);
    printf("\n\r");
#endif

#if (MEASURE_ACTIVE == 1)
#ifdef PCF_USE_CLUSTER
    timerBuffer_c[Timerindex_c++] = (Timer_Data_t){'W', get_timer_cnt_cl(timer0_id)};
#else
    timerBuffer[Timerindex++] = (Timer_Data_t){'W', lMeasureReadCsr(MEASURE_ZEROING)};
#endif
#endif

/*-----------------------------------------------*/
/************* 7: Reduce Temperature *************/
/*-----------------------------------------------*/
#ifdef USE_CTRL_STRCT_VB
    for (varFor i = 0; i < p_config_sys->num_cores; i++) {
        reduced_core_power[i] = p_values_ctrl->target_core_power[i];
        p_values_ctrl->target_core_power[i] = og_core_power[i];
    }
#endif

#if (!defined(USE_CTRL_STRCT_INVERSE) && \
     !defined(USE_CTRL_FUZZY)) //(like this becasue both VB and Direct Needs it)
    p.f = fComputeTemperatureReduction;
    p.total_iterations = p_config_sys->num_cores;
    p.exec_time = 1;
    p.ptr_args = NULL;
    p.return_value = NULL;
    MACRO_CP_CLUSTER_PARALLELIZE(&p);

#if (MEASURE_ACTIVE == 1)
#ifdef PCF_USE_CLUSTER
    timerBuffer_c[Timerindex_c++] = (Timer_Data_t){'T', get_timer_cnt_cl(timer0_id)};
#else
    timerBuffer[Timerindex++] = (Timer_Data_t){'T', lMeasureReadCsr(MEASURE_ZEROING)};
#endif
#endif

#endif //(!defined(USE_CTRL_STRCT_INVERSE) && !defined(USE_CTRL_FUZZY))

#ifdef USE_CTRL_STRCT_VB
    for (varFor i = 0; i < p_config_sys->num_cores; i++) {
        if (reduced_core_power[i] < p_values_ctrl->target_core_power[i]) {
            p_values_ctrl->target_core_power[i] = reduced_core_power[i];
        }
    }
#endif

#ifdef CTRL_DEBUG
    printf("P3: ");
    printFloat(p_values_ctrl->target_core_power[0]);
    printf("\n\r");
#endif

    /*-----------------------------------------------*/
    /************** 8: Compute Frequency *************/
    /*-----------------------------------------------*/

    vComputeFrequency(&l_cti);

#ifdef CTRL_DEBUG
    printf("F2: ");
    printFloat(p_values_ctrl->op.computed_core_frequency[0]);
    printf("\n\r");

// printf("Tf2: ");
// printFloat(p_input->target_freq[0]);
// printf("\n\r");
#endif

#if (MEASURE_ACTIVE == 1)
#ifdef PCF_USE_CLUSTER
    timerBuffer_c[Timerindex_c++] = (Timer_Data_t){'C', get_timer_cnt_cl(timer0_id)};
    stop_timer_cl(timer0_id);
#else
    timerBuffer[Timerindex++] = (Timer_Data_t){'C', lMeasureReadCsr(MEASURE_ZEROING)};
#endif
#endif

// TODO: FIX THIS
/*** COPY BACK ***/
#ifdef PCF_USE_CLUSTER
    if (parallel_num > 0) // If we are on the cluster //TBC: if there exist a better function
    {
        /** DMA **/
        unsigned int dma_id_value2 =
            pulp_cl_idma_memcpy(tp->cl_ptr, &l_values_ctrl, sizeof(ctrl_values_table_t));

#if (MEASURE_ACTIVE == 1)
        /** DMA **/
        unsigned int dma_id_value3 =
            pulp_cl_idma_memcpy((&timerBuffer + sizeof(Timer_Data_t) * Timerindex), &timerBuffer_c,
                                sizeof(Timer_Data_t) * (Timerindex_c - 1));
#endif

        // TBD: here dma wasted?

        // TBD: here dma wasted?
        plp_cl_dma_wait(dma_id_value2);
        // p_values_ctrl = &l_values_ctrl;

#if (MEASURE_ACTIVE == 1)
        plp_cl_dma_wait(dma_id_value3);
        // p_values_ctrl = &l_values_ctrl;
        Timerindex += Timerindex_c;
#endif
    }
#endif

    // TODO: fix this!!!!1
    // for (varFor domain = 0; domain < p_config_sys->num_domains; domain++)
    //{
    //	p_values_ctrl->op.computed_domain_voltage[domain] = computed_domain_voltage[domain];
    //}

    // TODO: fix and optimize this
    // if (bWriteGlobalVariable(&c_parameters_table, G_CTRL_PARAMETER_TABLE, GLOBAL_WRITE_ALL,
    // sizeof(c_parameters_table), c_task_id) != PCF_TRUE)
    //{
    // TODO

    //	#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //	printf("bWriteGlobalVariable issue: l_ctrl_inputs_table, id: %d\n\r", c_task_id);
    //	#endif
    //}
}

varValue fCheckFrequencies(varFor i_start, varFor i_end, varFor i_increment,
                           struct ctrl_task_index *tptr, void **args) {
    // Check Freq:
    for (varFor i = i_start; i < i_end; i += i_increment) {
        if (tptr->values->op.computed_core_frequency[i] >
            tptr->values->op.store_max_freq[tptr->config_sys->core_domain_id[i]])
        // tptr->config_sys->voltage_table[c_parameters_table.computed_domain_voltage[tptr->config_sys->core_domain_id[i]]*3+2])
        {
            /*
            #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                                    printf("[%d]: - OGF: ", i);
                                    printFloat(tptr->values->op.computed_core_frequency[i]);
                                    printf(" - NF: ");
                                    printFloat(
            tptr->values->op.store_max_freq[tptr->config_sys->core_domain_id[i]]);
                                    printf("\n\r");
            #endif
            */
            tptr->values->op.computed_core_frequency[i] =
                tptr->values->op.store_max_freq[tptr->config_sys->core_domain_id[i]];
            tptr->values->freq_red_map[i] |= BM_FRM_VDD_MAX_FREQ_LIMIT;
        }
    }

    return 0;
}

varValue fMultiInputProcess(varFor i_start, varFor i_end, varFor i_increment,
                            struct ctrl_task_index *tptr, void **args) {
    // TODO: Perc or Ceff should have been fetched before

    for (varFor core = i_start; core < i_end; core += i_increment) {
// TODO: Ceff should be already setup!
#ifdef USE_INSTRUCTIONS_COMPOSITION
        varValue power_formula_coeff[PCF_MODEL_COEFF];
        for (varFor k = 0; k < PCF_MODEL_COEFF; k++) {
            power_formula_coeff[k] = 0;

            for (varFor j = 0; j < tptr->config_sys->num_wl_states; j++) {
                // power_formula_coeff[k] += i_ctrl_parameter_table->power_formula_coeff[ j ][k] *
                // (varValue)i_ctrl_parameter_table->perc_workload[i][j] / VD_100_PERC;
                power_formula_coeff[k] += tptr->config_sys->base_power_coeff[j][k] *
                                          tptr->inputs->msr.perf.perc[core][j] / VD_100_PERC;
            }

            // TODO
            tptr->inputs->msr.perf.ceff[core] = power_formula_coeff[1];
        }
#endif
        // TODO: move after ceff computation and do it inside Ceff.
        tptr->values->ip.workload[core] *= (1.0f - 0.4f);
        tptr->values->ip.workload[core] += 0.4f * (varValue)tptr->inputs->msr.perf.ceff[core];

        // TODO can I optimize this?
        tptr->values->core_frequency[core] = tptr->inputs->cmd.target_freq[core];
        /*
        printf("cf: ");
        printFloat(tptr->values->core_frequency[core]);
        printf("\n\r");
        */
    }

    if (tptr->values->ctrl_counter % tptr->config_ctrl->therm.der_mul == 0) {
        for (varFor core = i_start; core < i_end; core += i_increment) {
            tptr->values->ip.temp_der[core] =
                tptr->inputs->msr.temp.core[core] - tptr->values->ip.prev_temp[core];
            tptr->values->ip.prev_temp[core] = tptr->inputs->msr.temp.core[core];
        }
    }

    return 0;
}

inline void vPowerBudgetProcess(struct ctrl_task_index *tptr) {
    if (tptr->values->ip.prev_power_budget != tptr->inputs->cmd.total_power_budget) {
        if (tptr->config_ctrl->conf.variable_voltage_solution == VV_MOVING_AVERAGE)
        // if (tptr->values->ip.prev_power_budget  < tptr->inputs->cmd.total_power_budget)
        // //TBC: only <, because this fix working good on increasing the power budget, but bad
        // on decreasing it.
        {
            tptr->values->ma.alpha_counter = PW_CHANGED_ALPHA_COUNTER; // 16
            // tptr->values->ma.alpha = 0.8f; //TODO: define a global value: //0.4

            // zero all the things
            /*
            //This is bad, cuz it make Voltage to jump at max value. Instead I should modulate alpha
            to make it change quicklier
            for (varFor i = 0; i < tptr->config_sys->num_cores; i++)
            {
                    tptr->values->freq_diff_red[i] = 0;
            }
            */
        }
        //}

        tptr->values->ip.power_budget_changed = PCF_TRUE; // TODO move to imp
        tptr->values->ip.prev_power_budget = tptr->inputs->cmd.total_power_budget;
    } else {
        tptr->values->ip.power_budget_changed = PCF_FALSE;
    }
    //}
}

inline void vPowerConsumptionAdaptation(struct ctrl_task_index *tptr) {
    // TODO MOVE THIS INTO FAST TASK
    bImpReadPowerMeasure(&tptr->inputs->msr.pw);

    /*
varValue estimated_reduced_total_power;

    tptr->fp->f = fComputeEstimatedTotalPower;
    tptr->fp->total_iterations = tptr->config_sys->num_cores;
    tptr->fp->exec_time = 1;
    tptr->fp->ptr_args = NULL;
    tptr->fp->return_value = (void*)&estimated_reduced_total_power;
    MACRO_CP_CLUSTER_PARALLELIZE(tptr->fp);
    */

    varValue total_power_adaptation_delta =
        tptr->inputs->msr.pw.total - tptr->values->ip.prev_tot_power[0];
    varValue alpha_salita = 0.04f; // TODO: define and TUNE! these coefficients.
    varValue alpha_discesa = 0.08f;
    varValue total_power_adaptation_alpha = 0;

    if (total_power_adaptation_delta > 0) {
        total_power_adaptation_alpha = alpha_discesa;
    } else {
        total_power_adaptation_alpha = alpha_salita;
    }
    // if (tptr->values->ma.power_budget_changed)
    //{
    //	total_power_adaptation_alpha += 0.2;
    //}
    if (tptr->values->ma.alpha_counter > 0) {
        // TODO: formula of parabola thought 2 points and and line
        // for the moment linear //TODO GLOBAL ETC
        // TODO: also maybe interpolate the line: e.g. if the 2 pw budget are very different the 0.8
        // is 0.95, if they are
        //		close it is better 0.5 or lower, etc.
        total_power_adaptation_alpha = (0.8f - total_power_adaptation_alpha) /
                                           PW_CHANGED_ALPHA_COUNTER *
                                           tptr->values->ma.alpha_counter +
                                       total_power_adaptation_alpha;
    }

    tptr->values->ip.total_power_adaptation_term *= (VD_ONE - total_power_adaptation_alpha);
    tptr->values->ip.total_power_adaptation_term +=
        total_power_adaptation_delta * total_power_adaptation_alpha;

    tptr->values->ip.prev_tot_power[0] = tptr->values->ip.prev_tot_power[1];
}

varValue fComputeEstimatedTotalPower(varFor i_start, varFor i_end, varFor i_increment,
                                     struct ctrl_task_index *tptr, void **args) {
    varValue estimated_total_power = 0;

    for (varFor i = i_start; i < i_end; i += i_increment) {
        if (tptr->config_ctrl->conf.variable_voltage_solution == VV_MOVING_AVERAGE) {
            if (tptr->values->ma.og_freq[i] == 0) {
                tptr->values->ma.og_freq[i] = tptr->config_sys->core_min_frequency;
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("Error, og freq core %d = 0\n\r", i);
#endif
            }

            // TODO: use formula below for int?
            // estimated_total_power += ( target_core_power[i] *
            // (vv_moving_average_og_freq[i]*VD_100_PERC / computed_core_frequency[i]) ) /
            // VD_100_PERC;
            varValue freq_reduction_coeff =
                tptr->values->op.computed_core_frequency[i] / tptr->values->ma.og_freq[i];
            if (freq_reduction_coeff <= 0.01f) // TODO: check values
            {
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("Error < 0: ");
                printFloat(freq_reduction_coeff);
                printf(" - core: %d - ogf: ", i);
                printFloat(tptr->values->ma.og_freq[i]);
                printf(" - af: ");
                printFloat(tptr->values->op.computed_core_frequency[i]);
                printf("\n\r");
#endif
                freq_reduction_coeff = 0.01f;
            } else if (freq_reduction_coeff > VD_ONE) {
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("Error > 0: ");
                printFloat(freq_reduction_coeff);
                printf(" - core: %d - ogf: ", i);
                printFloat(tptr->values->ma.og_freq[i]);
                printf(" - af: ");
                printFloat(tptr->values->op.computed_core_frequency[i]);
                printf("\n\r");
#endif
                freq_reduction_coeff = VD_ONE;
            }

            estimated_total_power += tptr->values->target_core_power[i] * freq_reduction_coeff;
        } else {
            estimated_total_power += tptr->values->target_core_power[i];
        }
    }

    // printf("est: ");
    // printFloat(estimated_total_power);
    // printf("\n\r");

    return estimated_total_power;
}

varValue fCorePowerComputation(varFor i_start, varFor i_end, varFor i_increment,
                               struct ctrl_task_index *tptr, void **args) {

    varValue *iFreq = (varValue *)args[0];
    varValue total_power = 0;

    // TBD: here we can also pass the pointer to the array and call the function only 1 time. The
    // function will do for and sum.
    for (varFor i = i_start; i < i_end; i += i_increment) {
        varValue power_formula_coeff[PCF_MODEL_COEFF];

        /*#ifdef USE_INSTRUCTIONS_COMPOSITION
        for (varFor k = 0; k < PCF_MODEL_COEFF; k++)
        {
                power_formula_coeff[k] = 0;

                for (varFor j = 0; j < tptr->config_sys->num_wl_states; j++)
                {
                        //power_formula_coeff[k] += i_ctrl_parameter_table->power_formula_coeff[ j
    ][k] * (varValue)i_ctrl_parameter_table->perc_workload[i][j] / VD_100_PERC;
                        power_formula_coeff[k] += tptr->config_sys->base_power_coeff[ j ][k] *
    tptr->inputs->msr.perf.perc[i][j] / VD_100_PERC;
                }

    //TODO
    tptr->inputs->core_ceff[i] = power_formula_coeff[1];
        }
        #else*/
        power_formula_coeff[0] = tptr->config_sys->base_power_coeff[0][0];
        power_formula_coeff[1] = tptr->values->ip.workload[i];
        // #endif

        varValue core_power = lMathPowerCompute(
            iFreq[i], tptr->values->op.computed_domain_voltage[tptr->config_sys->core_domain_id[i]],
            power_formula_coeff);

        /*
        if (core_power < tptr->values->target_core_power[i] - tptr->config_ctrl->max_pw_diff)
        {
                tptr->values->th.ctrl_cmd[i] = core_power / tptr->values->target_core_power[i];
        }
        else if (core_power > tptr->values->target_core_power[i] + tptr->config_ctrl->max_pw_diff)
        {
                tptr->values->th.ctrl_cmd[i] = core_power / tptr->values->target_core_power[i];
        }
        else
        {
                tptr->values->th.ctrl_cmd[i] = VD_ZERO;
        }
        */

        // test_on_number
        if (core_power < tptr->config_sys->core_idle_power) {
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            // printf("Number issue, computed_core_power core %d: %f\n\r", i, core_power[i]);
            printf("Number issue, computed_core_power core %d: ", i);
            printFloat(core_power);
            printf(" - F: ");
            printFloat(tptr->values->core_frequency[i]);
            printf(" - V: ");
            printFloat(
                tptr->values->op.computed_domain_voltage[tptr->config_sys->core_domain_id[i]]);
            printf(" - CI: ");
            printFloat(tptr->values->ip.workload[i]);
            printf("\n\r");
#endif

            core_power = tptr->config_sys->core_idle_power;
        } else if (core_power > tptr->config_sys->core_max_power_single) {
            core_power = tptr->config_sys->core_max_power_single;

            // Bitmap of Frequency Reduction.
            tptr->values->freq_red_map[i] |= BM_FRM_MAX_SINGLE_POW_SAT;
        }

        tptr->values->target_core_power[i] = core_power;

        total_power += core_power;

        // TODO: for quadrants, we should change this to reflect quadrants disposition so we can add
        // quadrant power also inside this for

        // Saturation check for single Core PowerMax Margin
    }

    return total_power;
}

varValue fVoltageCastVote(varFor i_start, varFor i_end, varFor i_increment,
                          struct ctrl_task_index *tptr, void **args) {
    uint16_t *i_voltage_voting = (uint16_t *)args[0];

    for (varFor i = i_start; i < i_end; i += i_increment) {
        varValue freq = tptr->inputs->cmd.target_freq[i];

        if (tptr->config_ctrl->conf.variable_voltage_solution == VV_MOVING_AVERAGE) {
            freq -= tptr->values->ma.freq_point[i];
        }

        uint16_t domain = tptr->config_sys->core_domain_id[i];
        varFor index = 0;
        while (index < tptr->config_sys->num_voltage_levels) {
            if (freq <= tptr->config_sys->voltage_table[index * 3 + 2]) // TODO here just lloking at
                                                                        // the min freq, not
                                                                        // considering voltage
                                                                        // overheads or other
                                                                        // changes
            {
                i_voltage_voting[index + domain * tptr->config_sys->num_voltage_levels]++;
                break; // index = tptr->config_sys->num_voltage_levels+1;
            } else
                index++;
        }
        // checks
        if (index == tptr->config_sys->num_voltage_levels) {
            // TODO error > max!

#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
// printf("Error voltage choice core %d > max\n\r", i);
#endif

            // TODO: put target at max

            // casting vote:
            i_voltage_voting[index - 1 + domain * tptr->config_sys->core_domain_id[i]]++;
        }

        /*
        if (i==2){
        printf("freq [%d]: ", i);
        printFloat(freq);
        printf(" - index: %d\n\r", index);
        }
        */
    }

    return 0;
}

varValue fVoltageChoice(varFor i_start, varFor i_end, varFor i_increment,
                        struct ctrl_task_index *tptr, void **args) {
    // TODO here I assumed performance level is already the percentage
    // TODO: voltage decision is not done correctly!! It may want to select a Voltage that is not
    // the one of ANY core,
    //		but it is still in an intermediate point, e.g. 8 cores at 0.5V, 1core at 1.2V, now
    //it is
    // either 0.5 or 1.2, but
    //		I may like to do 0.75V for example
    // instead of doing votes/N -> perc/100; I convert in votex*100 -> perc*N, so I have no div and
    // no issue with int numbers.

    //  uint32_t performance_perc = tptr->inputs->cmd.perfmormance_level;

    uint16_t *i_voltage_voting = (uint16_t *)args[0];

    for (varFor i = i_start; i < i_end; i += i_increment) {
        uint32_t accumulator = 0;
        // uint16_t domain = i_ctrl_parameter_table->domain[i];

        varFor index = tptr->config_sys->num_voltage_levels - 1;
        while (index >= 0) {
            accumulator +=
                (uint32_t)i_voltage_voting[index + i * tptr->config_sys->num_voltage_levels] * 100;

            /*
            if (i==2){
            printf("Accumulator: %d, vv: %d\n\r", accumulator, i_voltage_voting[index +
            i*tptr->config_sys->num_voltage_levels]);
            }
            */

            if (accumulator >=
                tptr->inputs->cmd.perfmormance_level * tptr->config_sys->num_cores_per_domain[i]) {
                tptr->values->op.computed_domain_voltage[i] =
                    tptr->config_sys->voltage_table[index * 3 + 0]; // TODO here just lloking at the
                                                                    // min freq, not considering
                                                                    // voltage overheads or other
                                                                    // changes
                tptr->values->op.store_max_freq[i] = tptr->config_sys->voltage_table[index * 3 + 2];
                tptr->values->op.store_voltage_level[i] = index;
                break; // index = -1;
            } else
                index--;
        }
        // checks
        if (index < 0) {
            // TODO error

#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
// printf("Error voltage choice domain %d < min\n\r", i);
#endif

            // TODO: put target at min

            tptr->values->op.computed_domain_voltage[i] =
                tptr->config_sys->voltage_table[0 * 3 + 0]; // TODO here just lloking at the min
                                                            // freq, not considering voltage
                                                            // overheads or other changes
            tptr->values->op.store_max_freq[i] = tptr->config_sys->voltage_table[0 * 3 + 2];
            tptr->values->op.store_voltage_level[i] = 0;
        }

        /*
        if (i==2){
        printf("freq [%d]: ", i);
        printFloat(tptr->values->op.store_max_freq[i] );
        printf(" V: ");
        printFloat(tptr->values->op.computed_domain_voltage[i]);
        printf(" - index: %d\n\r", index);
        }
        */
    }

#ifdef FIX_VOLTAGE
    for (varFor i = 0; i < tptr->config_sys->num_domains; i++) {
        tptr->values->op.computed_domain_voltage[i] =
            tptr->config_sys->voltage_table[(6 - 1) * 3 + 0];
        tptr->values->op.store_max_freq[i] = tptr->config_sys->voltage_table[(6 - 1) * 3 + 2];
    }
#endif // FIX_VOLTAGE

    return 0;
}

varValue fFVpreAdaptation(varFor i_start, varFor i_end, varFor i_increment,
                          struct ctrl_task_index *tptr, void **args) {
    for (varFor i = i_start; i < i_end; i += i_increment) {
        int indexD = 0;
        while ((tptr->values->ip.temp_der[i] > tptr->config_ctrl->therm.fuzzy_der_lim[indexD]) &&
               (indexD < 4)) // TODO make it parametric
        {
            indexD++;
        }
        int indexT = 0;
        while ((tptr->inputs->msr.temp.core[i] >
                tptr->config_ctrl->therm.fuzzy_temp_lim[indexT] + 273.15) &&
               (indexT < 4)) // TODO make it parametric
        {
            indexT++;
        }

        /*
        if (i==2) {
        printf("Core %d: VCred: %d - fuzzy value: %d\n\r", i, tptr->values->th.fuzzy_vcred[i],
        tptr->config_ctrl->therm.fuzzy_value_table[indexT*5+indexD]);
        printf("\t indexT: %d - indexD: %d, T: ", indexT, indexD);
        printFloat(tptr->inputs->msr.temp.core[i]-273.15);
        printf(" der: ");
        printFloat(tptr->values->ip.temp_der[i]);
        printf("\n\r");
        }
        */

        // be careful! Matlab index matrices vertically (opposite to C)
        tptr->values->th.fuzzy_vcred[i] +=
            tptr->config_ctrl->therm.fuzzy_value_table[indexT * 5 + indexD]; // TODO parametrize the
                                                                             // 5.

        // Saturate if no turbo
        // TODO: add parametrization
        if (tptr->values->th.fuzzy_vcred[i] > 0) {
            tptr->values->th.fuzzy_vcred[i] = 0;
        }
    }

    return 0;
}

inline void vFVdomainAdaptation(struct ctrl_task_index *tptr) {
    // varValue *core_vcred = (varValue*)args[0];
    varFor min_domain_vcred[PCF_DOMAINS_MAX];
    // TODO can I optimize this?
    for (varFor domain = 0; domain < tptr->config_sys->num_domains; domain++) {
        min_domain_vcred[domain] = 20; // TODO
    }
    for (varFor core = 0; core < tptr->config_sys->num_cores; core++) {
        if (tptr->values->th.fuzzy_vcred[core] <
            min_domain_vcred[tptr->config_sys->core_domain_id[core]]) {
            min_domain_vcred[tptr->config_sys->core_domain_id[core]] =
                tptr->values->th.fuzzy_vcred[core];
        }
    }

    for (varFor domain = 0; domain < tptr->config_sys->num_domains; domain++) {
        /*
        if (domain==2)
        {
        printf("dom: %d, og V: ", domain);
        printFloat(tptr->values->op.computed_domain_voltage[domain]);
        printf(" - level: %d, red: %d, new V: ", tptr->values->op.store_voltage_level[domain],
        min_domain_vcred[domain] / 2);
        }
        */

        // Reduce V
        tptr->values->op.store_voltage_level[domain] +=
            (min_domain_vcred[domain] / 2); // TODO parametrize / 2
        if (tptr->values->op.store_voltage_level[domain] < 0) {
            tptr->values->op.store_voltage_level[domain] = 0;
            // TODO trigger something!
        }

        tptr->values->op.computed_domain_voltage[domain] =
            tptr->config_sys->voltage_table[tptr->values->op.store_voltage_level[domain] * 3 + 0];
        tptr->values->op.store_max_freq[domain] =
            tptr->config_sys->voltage_table[tptr->values->op.store_voltage_level[domain] * 3 + 2];

        /*
        if (domain ==2)
        {
                printFloat(tptr->config_sys->voltage_table[tptr->values->op.store_voltage_level[domain]*3+0]);
                printf("\n\r");
        }
        */

        // Reduce F
        varValue F_adaptation = tptr->values->op.store_max_freq[domain] +
                                (varValue)(min_domain_vcred[domain] % 2) *
                                    0.05; // TODO parametrize both 2 (with above) and 0.05

        /*
        printf(" F: ");
        printFloat(F_adaptation);
        //printf("\n\r");
        */

        for (varFor core = 0; core < tptr->config_sys->num_cores; core++) {
            if (tptr->config_sys->core_domain_id[core] == domain) {
                tptr->values->core_frequency[core] =
                    tptr->values->core_frequency[core] > F_adaptation
                        ? F_adaptation
                        : tptr->values->core_frequency[core];

                /*
                printf(" Fc: ");
                printFloat(tptr->values->core_frequency[core]);
                printf("\n\r");
                */
            }
        }
    }
}

varValue fDummyPowerReduction(varFor i_start, varFor i_end, varFor i_increment,
                              struct ctrl_task_index *tptr, void **args) {
    varValue *ptr_cast = (varValue *)args[0];
    varValue delta = *ptr_cast;
    ptr_cast = (varValue *)args[1];
    varValue tot_power = *ptr_cast;

    varValue perc = delta / tot_power;

    for (varFor i = i_start; i < i_end; i += i_increment) {
        varValue reduction = tptr->values->target_core_power[i] * perc;
        tptr->values->target_core_power[i] -= reduction;

// TODO: Canc:
#ifdef ALPHA_DEBUG
        varValue *mem_address = NULL;
        mem_address = (varValue *)IMP_ADR_OUT_FIRST_ALPHA;
        size_t address_num_of_bytes = sizeof(varValue);
        // TODO: Compiler doesn't work properly: it multiply everything for 4.
        address_num_of_bytes = 1;

        mem_address += i * address_num_of_bytes;
        if (mem_address > (varValue *)IMP_ADR_OUT_ALPHA_LAST) {
            printf("Error writing alpha\r\n");
        }

        *mem_address = perc;

        mem_address = (varValue *)IMP_ADR_OUT_FIRST_REDPW;

        mem_address += i * address_num_of_bytes;
        if (mem_address > (varValue *)IMP_ADR_OUT_REDPW_LAST) {
            printf("Error writing alpha\r\n");
        }

        *mem_address = tptr->values->target_core_power[i];
#endif // ALPHA_DEBUG
    }

    return 0;
}

inline void vAlphaPowerReduction(varValue i_delta_power, varValue est_total_pw,
                                 struct ctrl_task_index *tptr) {

    varValue accumulation_Alpha = 0;
    varValue accumulation_pu = 0;
    varValue Alpha[PCF_CORES_MAX];
    varBool_e min_pw_reached[PCF_CORES_MAX];
    varValue pw_storage = -i_delta_power;

    varValue safety_value = -1.0e-3;
    varFor iterations = 0;
    varFor break_while = 0;

    void *l_ptr_args[5]; // TODO becareful when choosing this number.

    l_ptr_args[0] = (void *)&pw_storage;
    l_ptr_args[1] = (void *)Alpha;
    l_ptr_args[2] = (void *)min_pw_reached;
    l_ptr_args[3] = (void *)&accumulation_Alpha;

    // TODO: find a way to initialize this better:
    for (varFor core = 0; core < tptr->config_sys->num_cores; core++) {
        min_pw_reached[core] = PCF_FALSE;
    }

    // Here we limit the iteration to an upper bound to define the maximum time, for scheduling
    // purposes
    while ((pw_storage < safety_value) && (iterations < 3) &&
           (break_while < tptr->config_sys->num_cores)) // TODO & TBD parameter 3
    {
        /*
        printf("pw_storage [%d]-%d: ", iterations, break_while);
        printFloat(pw_storage);
        printf("\n\r");
        if (iterations>0)
        {
                printf("pw_storage [%d]-%d: ", iterations, break_while);
                printFloat(pw_storage);
                printf("\n\r");
        }
        */

        // Compute Alpha
        tptr->fp->f = fComputeAlpha;
        tptr->fp->total_iterations = tptr->config_sys->num_cores;
        tptr->fp->exec_time = 1;
        tptr->fp->ptr_args = l_ptr_args;
        tptr->fp->return_value = &accumulation_Alpha;
        MACRO_CP_CLUSTER_PARALLELIZE(tptr->fp);

        if (tptr->config_ctrl->conf.use_freq_binding == PCF_TRUE) {
            // TODO: THIS IS WRONG, NEED A DIFFERENT ALGORITHM
            // vComputeAlphaBindings(pp, tptr);
        }

        // Normalize Alpha
        tptr->fp->f = fNormalizeApplyAlpha;
        tptr->fp->total_iterations = tptr->config_sys->num_cores;
        tptr->fp->exec_time = 1;
        tptr->fp->ptr_args = l_ptr_args;
        tptr->fp->return_value = &accumulation_pu;
        MACRO_CP_CLUSTER_PARALLELIZE(tptr->fp);

        // Apply Alpha Reduction
        /*
        tptr->fp->f = fApplyAlphaReduction;
        tptr->fp->total_iterations = tptr->config_sys->num_cores;
        tptr->fp->exec_time = 1;
        l_ptr_args[0] = (void*)&pw_storage;
        l_ptr_args[1] = (void*)Alpha;
        tptr->fp->ptr_args = l_ptr_args;
        tptr->fp->return_value = NULL;
        MACRO_CP_CLUSTER_PARALLELIZE(tptr->fp);
        */

        pw_storage = est_total_pw - accumulation_pu - i_delta_power;
        break_while = 0;
        // TODO: update this and make it a bitmap so I can do all of this in 1 operation
        for (varFor core = 0; core < tptr->config_sys->num_cores; core++) {
            if (min_pw_reached[core] == PCF_TRUE) break_while += 1;
        }

        /*
        printf("[%d] acc pu: ", iterations);
        printFloat(accumulation_pu);
        printf(" - pw_storage: ");
        printFloat(pw_storage);
        printf(" - bw: %d", break_while);
        printf("\n\r");
        */

        accumulation_Alpha = 0;
        accumulation_pu = 0;
        iterations++;
    }

    // return TotalPower;
}

varValue fComputeAlpha(varFor i_start, varFor i_end, varFor i_increment,
                       struct ctrl_task_index *tptr, void **args) {
    varValue *ptr_cast = (varValue *)args[0];
    varValue delta_power = *ptr_cast;
    varValue *Alpha = (varValue *)args[1];
    varBool_e *mpwr = (varBool_e *)args[2];

    varValue accumulation_Alpha = 0;

    varValue safety_margin = 5.0f; // TODO parametrize

    for (varFor i = i_start; i < i_end; i += i_increment) {
        // test_on_number //MUST do always, not only when test on numbers, and it is not a ToN
        // Error!
        if (mpwr[i] == PCF_TRUE) {
            Alpha[i] = 0;
        } else // nominal Alpha computation
        {
            if ((tptr->config_sys->core_critical_temperature -
                 tptr->config_ctrl->therm.pid_temp_margin - tptr->inputs->msr.temp.core[i] -
                 safety_margin) >= 0) {
                Alpha[i] = 1.0f / (tptr->config_sys->core_critical_temperature -
                                   tptr->inputs->msr.temp.core[i]);
            } else {
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
// printf("Number issue, i_measured_temperature > l_core_critical_temperature, core %d: %f\n\r", i,
// i_measured_temperature[i]);
// printf("Number issue, i_measured_temperature > l_core_critical_temperature, core %d: ", i);
// printFloat(tptr->inputs->msr.temp.core[i]);
// printf("\n\r");
#endif

                Alpha[i] = 1.0f / (tptr->config_ctrl->therm.pid_temp_margin);

                // Test on Alpha acceptability:
                // IT should not be done here, but below with the normalized Alpha. This is wrong
                /*
                if (delta_power*Alpha[i] > tptr->values->target_core_power[i] -
                tptr->config_sys->core_idle_power)
                {
                        Alpha[i] = (tptr->values->target_core_power[i] -
                tptr->config_sys->core_idle_power) / delta_power;

                        //TODO add "not reduction" in a bin, so I know I have to reduce more!
                }
                */

                // test_on_number
                if (Alpha[i] <= 0) {
                    /* TODO:
                    //if (g_CodeConfigTable.use_error_map == PCF_TRUE)
                    ErrorMap |= BM_ERROR_NUMBERS_VALUE;
                    #endif
                    */

#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                    // printf("Number issue, Alpha<0, core %d: %f\n\r", i, Alpha[i]);
                    printf("Number issue, Alpha<0 (1), core %d: ", i);
                    printFloat(Alpha[i]);
                    printf("\n\r");
#endif

                    Alpha[i] = tptr->config_ctrl->pw.alpha_error_base_value; // TBD: we are whitin
                                                                             // an ERROR!!! do not
                                                                             // put 0.01!

                } else if (Alpha[i] > tptr->config_ctrl->pw.alpha_max_value) {
/* TODO:
//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
ErrorMap |= BM_ERROR_NUMBERS_VALUE;
#endif
*/
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                    // printf("Number issue, Alpha>1, core %d: %f\n\r", i, Alpha[i]);
                    printf("Number issue, Alpha>1, core %d: ", i);
                    printFloat(Alpha[i]);
                    printf("\n\r");
#endif

                    Alpha[i] = tptr->config_ctrl->pw.alpha_max_value;
                }
            }
        }

        accumulation_Alpha += Alpha[i];
        // TODO: for quadrants, we should change this to reflect quadrants disposition so we can add
        // quadrant power also inside this for
    }

    return accumulation_Alpha;
}

inline void vComputeAlphaBindings(struct ctrl_task_index *tptr) {
    // TBU: This uint16+ is VERY delicate, it has to change with the
    //	typedef struct _ctrl_parameter_table, so it could need a better design

    // varCoreBinding* i_core_binding_vector = i_ctrl_parameter_table->core_binding_vector;
    // varValue min_group_alpha[(PCF_CORES_MAX / 2) + 1]; // +1 cuz of 0

    // varValue accumulation_Alpha = 0;

    // TODO: Optimize This
    // Initialization
    // for (varFor i = 0; i < (tptr->config_sys->num_cores / 2) +1; i++ )
    //     min_group_alpha[i] = VD_ONE;

    // // Scan
    // for (varFor i = 0; i < tptr->config_sys->num_cores; i++)
    // {
    //     if (i_core_binding_vector[i] != 0)
    //     {
    //         //test_on_number
    //         if ((i_core_binding_vector[i] > (tptr->config_sys->num_cores/2)) /*||
    // (i_core_binding_vector[i] < 0) since it is unsigned, is never <0*/)
    //         {
    //             //TODO
    //             //ErrorMap |= BM_ERROR_NUMBERS_VALUE;
    //             #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //             printf("Number issue, i_core_binding_vector>tptr->config_sys->num_cores/2, core
    // %d: %d\n\r", i, i_core_binding_vector[i]);
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
    // for (varFor i = 0; i < tptr->config_sys->num_cores; i++)
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

varValue fNormalizeApplyAlpha(varFor i_start, varFor i_end, varFor i_increment,
                              struct ctrl_task_index *tptr, void **args) {
    varValue *ptr_cast = (varValue *)args[0];
    varValue i_delta_power = *ptr_cast;
    varValue *Alpha = (varValue *)args[1];
    varBool_e *mpwr = (varBool_e *)args[2];
    ptr_cast = (varValue *)args[3];
    varValue i_accumulation_Alpha = *ptr_cast;

    varValue accumulation_pu = 0;

    for (varFor i = i_start; i < i_end; i += i_increment) {
        if (mpwr[i] != PCF_TRUE) {
            // Alpha Normalized. For optimization we use Alpha[i]
            Alpha[i] = Alpha[i] / i_accumulation_Alpha;
            // test_on_number
            if (Alpha[i] <= 0) {
/* TODO:
//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
ErrorMap |= BM_ERROR_NUMBERS_VALUE;
#endif
*/
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                // printf("Number issue, Alpha<0, core %d: %f\n\r", i, Alpha[i]);
                printf("Number issue, Alpha<0 (2), core %d: ", i);
                printFloat(Alpha[i]);
                printf(" - accum: ");
                printFloat(i_accumulation_Alpha);
                // printf(" - og: ");
                // printFloat(i_accumulation_Alpha*Alpha[i]);
                printf("\n\r");
#endif

                Alpha[i] = tptr->config_ctrl->pw.alpha_error_base_value; // TBD: we are whitin an
                                                                         // ERROR!!! do not put
                                                                         // 0.01!

            } else if (Alpha[i] > 1.0f) {
/* TODO:
//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
ErrorMap |= BM_ERROR_NUMBERS_VALUE;
#endif
*/
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                // printf("Number issue, Alpha>1, core %d: %f\n\r", i, Alpha[i]);
                printf("Number issue, Alpha>1, core %d: ", i);
                printFloat(Alpha[i]);
                printf(" - accum: ");
                printFloat(i_accumulation_Alpha);
                printf("\n\r");
#endif

                Alpha[i] = tptr->config_ctrl->pw.alpha_error_base_value; // TBD: we are whitin an
                                                                         // ERROR!!! do not put
                                                                         // 0.99!
            }

            // Update target_core_power
            tptr->values->target_core_power[i] += (Alpha[i] * i_delta_power);
            // test_on_number
            if (tptr->values->target_core_power[i] <= 0) {
/* TODO:
//if (g_CodeConfigTable.use_error_map == PCF_TRUE)
ErrorMap |= BM_ERROR_NUMBERS_VALUE;
#endif
*/
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                // printf("Number issue, o_reduced_core_power<0, core %d: %f\n\r", i,
                // o_reduced_core_power[i]);
                printf("Number issue, reduced target_core_power<0, core %d: ", i);
                printFloat(tptr->values->target_core_power[i]);
                printf(" og: ");
                printFloat(tptr->values->target_core_power[i] - (Alpha[i] * i_delta_power));
                printf(" alpha: ");
                printFloat(Alpha[i]);
                printf(" delta: ");
                printFloat(i_delta_power);
                printf("\n\r");
#endif

                mpwr[i] = PCF_TRUE;
                tptr->values->target_core_power[i] = tptr->config_sys->core_idle_power;

                // TODO:
                //	Do something also for mpwr, Alpha, and i_delta_power
            } else if (tptr->values->target_core_power[i] <=
                       tptr->config_sys->core_idle_power) // Do this after <=0 which is check on
                                                          // numbers. <idle is control check, values
                                                          // are ok!
            {
                mpwr[i] = PCF_TRUE;
                tptr->values->target_core_power[i] = tptr->config_sys->core_idle_power;
            }

            // todo
            tptr->values->freq_red_map[i] |= BM_FRM_ALPHA_RED;

// TODO: if no bindings (def) we should compute here the new estimated_total_power +=
// target_core_power[i];, otherwise do after bindings
// #ifndef EPI_CONFIG_FREQ_BINDING //TODO: remove this and remove the return.
// TotalPower += o_reduced_core_power[i];
// #endif

// TODO: Canc:
#ifdef ALPHA_DEBUG
            varValue *mem_address = NULL;
            mem_address = (varValue *)IMP_ADR_OUT_FIRST_ALPHA;
            size_t address_num_of_bytes = sizeof(varValue);
            // TODO: Compiler doesn't work properly: it multiply everything for 4.
            address_num_of_bytes = 1;

            mem_address += i * address_num_of_bytes;
            if (mem_address > (varValue *)IMP_ADR_OUT_ALPHA_LAST) {
                printf("Error writing alpha\r\n");
            }

            *mem_address = Alpha[i];

            mem_address = (varValue *)IMP_ADR_OUT_FIRST_REDPW;

            mem_address += i * address_num_of_bytes;
            if (mem_address > (varValue *)IMP_ADR_OUT_REDPW_LAST) {
                printf("Error writing alpha\r\n");
            }

            *mem_address = tptr->values->target_core_power[i];
#endif // ALPHA_DEBUG
        }
        // TODO: this should not be needed. I should remove it
        /*else
        {
                tptr->values->target_core_power[i] = tptr->config_sys->core_idle_power;
        }
        */

        accumulation_pu += tptr->values->target_core_power[i];
    }

    return accumulation_pu;
}

/*
varValue fAlphaDomain(varFor i_start, varFor i_end, varFor i_increment, struct ctrl_task_index*
tptr, void** args)
{

        return 0;
}
*/

varValue fComputeTemperatureReduction(varFor i_start, varFor i_end, varFor i_increment,
                                      struct ctrl_task_index *tptr, void **args) {
    for (varFor i = i_start; i < i_end; i += i_increment) {
        // tptr->values->target_core_power[i]; //TODO: Optimize this....
        varValue core_reduced_value;

        // PID
        core_reduced_value =
            lMathPidCompute(tptr->values->target_core_power[i],
                            tptr->inputs->msr.temp.core[i], i, &(tptr->values->th), tptr->config_ctrl /*, tptr->values->th.ctrl_cmd[i], tptr->values->ip.power_budget_changed*/);

        // TestOnNumber
        if ((core_reduced_value <= 0) ||
            (core_reduced_value > tptr->values->target_core_power[i])) {
// TODO c_error_map |= BM_ERROR_NUMBERS_VALUE;
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
            // printf("Number issue, target_core_power, core %d: %f\n\r", i, target_core_power[i]);
            printf("Number issue, target_core_power, core %d: ", i);
            printFloat(tptr->values->target_core_power[i]);
            printf("\n\r");
#endif

            tptr->values->target_core_power[i] =
                tptr->config_sys->core_idle_power; // TBD: maybe too much conservative in case > ??
                                                   // (consider still there is an error!)
        }

        if (core_reduced_value < tptr->values->target_core_power[i]) // TODO
        {
            tptr->values->freq_red_map[i] |= BM_FRM_PID_TEMP_RED;
        }

        tptr->values->target_core_power[i] = core_reduced_value;
    }

    return 0;
}

void vComputeFrequency(struct ctrl_task_index *tptr) {
    void *l_ptr_args[3]; // TODO becareful when choosing this number.

    // #ifdef USE_COUPLING_SOLUTION
    if (tptr->config_ctrl->conf.variable_voltage_solution == VV_COUPLING_SOLUTION) {
        struct coupling_p2f l_coupling_ctrl;
        l_coupling_ctrl.ntrial = 16;
        l_coupling_ctrl.tolf = 0.05;
        l_coupling_ctrl.tolx = 0.05;
        l_ptr_args[0] = (void *)&l_coupling_ctrl;
        // TBD: here MACRO_CP_CLUSTER_PARALLELIZE is not called
        fFrequencyCouplingTransformation(0, tptr->config_sys->num_domains, 1, tptr, l_ptr_args);
    } else if (tptr->config_ctrl->conf.variable_voltage_solution == VV_MOVING_AVERAGE) {
        tptr->fp->f = fFrequency2PowerConversion;
        tptr->fp->total_iterations = tptr->config_sys->num_cores;
        tptr->fp->exec_time = 1;
        tptr->fp->ptr_args = NULL;
        tptr->fp->return_value = NULL;
        MACRO_CP_CLUSTER_PARALLELIZE(tptr->fp);
    } else {
        // TODO

        printf("ERRORRRRRR \n\r"); // TODO
    }

    /*** Check Bindings between Cores ***/
    // TODO
    if (tptr->config_ctrl->conf.use_freq_binding ==
        PCF_TRUE) // TODO create a local variable. Also check the code.
    {
        // TODO
        // vCoreBindingReduction(computed_core_frequency, &c_values_table, &c_telemetry_table);
    }

    // #ifndef USE_COUPLING_SOLUTION
    if (tptr->config_ctrl->conf.variable_voltage_solution == VV_MOVING_AVERAGE) {
        // update freq_diff_red paramenter:
        // TODO: optimize for parallel
        varValue ma_alpha = tptr->config_ctrl->pw.moving_average_alpha;
        // WHOLE, for power budget change
        if (tptr->values->ma.alpha_counter > 0) {
            // TODO: formula of parabola thought 2 points and and line
            // for the moment linear //TODO GLOBAL ETC
            // TODO: also maybe interpolate the line: e.g. if the 2 pw budget are very different the
            // 0.8 is 0.95, if they are
            //         close it is better 0.5 or lower, etc.
            ma_alpha = (0.9f - tptr->config_ctrl->pw.moving_average_alpha) /
                           (PW_CHANGED_ALPHA_COUNTER)*tptr->values->ma.alpha_counter +
                       tptr->config_ctrl->pw.moving_average_alpha;

            tptr->values->ma.alpha_counter--; // After
        }
        for (varFor i = 0; i < tptr->config_sys->num_cores; i++) {
            varValue ma_alpha_c = ma_alpha;
            // for single TF change
            if (tptr->inputs->cmd.target_freq_changed[i] == PCF_TRUE) {
                // if (tptr->config_ctrl->variable_voltage_solution == VV_MOVING_AVERAGE)
                // if (tptr->values->ip.prev_power_budget  < tptr->inputs->cmd.total_power_budget)
                // //TBC: only <, because this fix working good on increasing the power budget, but
                // bad on decreasing it.
                //	{

                // c_parameters_table.tf_changed_alpha_counter = TF_CHANGED_ALPHA_COUNTER; //16
                tptr->values->ma.alpha_counter = TF_CHANGED_ALPHA_COUNTER; // 16
            }
            if (tptr->values->ma.alpha_counter >
                0) // c_parameters_table.tf_changed_alpha_counter > 0)
            {
                // TODO: formula of parabola thought 2 points and and line
                // for the moment linear //TODO GLOBAL ETC
                // TODO: also maybe interpolate the line: e.g. if the 2 pw budget are very different
                // the 0.8 is 0.95, if they are
                //         close it is better 0.5 or lower, etc.
                ma_alpha_c = (1.0f - ma_alpha_c) /
                                 (TF_CHANGED_ALPHA_COUNTER)*tptr->values->ma
                                     .alpha_counter // c_parameters_table.tf_changed_alpha_counter
                             + ma_alpha_c;

                // c_parameters_table.tf_changed_alpha_counter--; //After
                tptr->values->ma.alpha_counter--; // After
            }
            // Moving average to avoid oscillation
            // if instant instead, just use alpha = 1
            tptr->values->ma.freq_point[i] *= (VD_ONE - ma_alpha_c);
            // TODO: "with" Turbo Boost
            // tptr->values->ma.freq_point[i] +=
            //    (tptr->inputs->cmd.target_freq[i] - tptr->values->op.computed_core_frequency[i]) *
            // ma_alpha_c;
            // without:
            // TBD: here tptr->values->core_frequency or tptr->inputs->cmd.target_freq?
            tptr->values->ma.freq_point[i] += ((tptr->inputs->cmd.target_freq[i] -
                                                tptr->values->op.computed_core_frequency[i]) > 0)
                                                  ? ((tptr->inputs->cmd.target_freq[i] -
                                                      tptr->values->op.computed_core_frequency[i]) *
                                                     ma_alpha_c)
                                                  : 0;
        }
    }

    varValue estimated_total_power;

    l_ptr_args[0] = (void *)tptr->values->op.computed_core_frequency;
    tptr->fp->f = fCorePowerComputation;
    tptr->fp->total_iterations = tptr->config_sys->num_cores;
    tptr->fp->exec_time = 1;
    tptr->fp->ptr_args = l_ptr_args;
    tptr->fp->return_value = &estimated_total_power;
    MACRO_CP_CLUSTER_PARALLELIZE(tptr->fp);

    tptr->values->ip.prev_tot_power[1] = estimated_total_power;
}

varValue fFrequencyCouplingTransformation(varFor i_start, varFor i_end, varFor i_increment,
                                          struct ctrl_task_index *tptr, void **args) {
    struct coupling_p2f *func_param = (struct coupling_p2f *)args[0];

#ifndef OPTIMIZATION_FUNC
    varValue x_solution[PCF_CORES_MAX];
#else
    varValue x_solution[PCF_CORES_MAX + 1];
#endif
    varValue func_param_wl[PCF_CORES_MAX];
    varValue func_param_Pc[PCF_CORES_MAX];
    // TODO find a way to avoid index?
    varFor core_index[PCF_CORES_MAX] = {0};

    varValue l_inf[PCF_CORES_MAX], l_sup[PCF_CORES_MAX];

    func_param->ptr_wl = func_param_wl;
    func_param->ptr_Pc = func_param_Pc;
    func_param->ptr_x = x_solution;
    func_param->ptr_cindex = core_index;
    func_param->ptr_target = tptr->values->target_core_power; // tptr->inputs->cmd.target_freq;
    func_param->ptr_lim_inf = l_inf;
    func_param->ptr_lim_sup = l_sup;
    func_param->usrfun = usrfunc;

    func_param->alpha_vdd = 0.3095f; // 0.2995f
    func_param->aoffset_vdd = 0.07;

    // #elif defined(USE_MONTE_CIMONE)
    func_param->k_vdd_stat = tptr->config_sys->base_power_coeff[0][0];
    func_param->k_stat = 0.52804f;

    // TODO: I don't think this is parallelizable? I should maybe set up something that understand
    // which is better
    // to parallelize based on the number of cores vs number of domains.
    for (varFor domain = i_start; domain < i_end; domain += i_increment) {
        // TODO: can I optimize this?
        // TODO how to split counter_core?
        // I can also not split the counter_core and split only the test bondaries

        void *l_ptr_args[3]; // TODO becareful when choosing this number.
        l_ptr_args[0] = (void *)&domain;
        l_ptr_args[1] = (void *)func_param;
        func_param->num_cores =
            lPopulateCouplingStructure(0, tptr->config_sys->num_cores, 1, tptr, l_ptr_args);

#ifdef OPTIMIZATION_FUNC
        func_param->ptr_x[func_param->num_cores] = 1.0; // TODO value, maybe use previous F(V)?
#endif

// printf("\n\rdomain: %d\n\r", domain);

// Compute NR
#ifdef USE_BISECT
#ifndef OPTIMIZATION_FUNC
        bisec(func_param->ntrial, x_solution, func_param->num_cores, func_param->tolx,
              func_param->tolf, l_inf, l_sup, func_param, pwrfunc); // TODO add fmin e fmax
#else // TODO: here BE CAREFUL BECAUSE N_CORE != Xn!!!!!!!!
        bisec(func_param->ntrial, x_solution, func_param->num_cores + 1, func_param->tolx,
              func_param->tolf, l_sup, func_param, usrfunc); // TODO add fmin e fmax
#endif
#else
        mnewt(func_param->ntrial, x_solution, func_param->num_cores, func_param->tolx,
              func_param->tolf, 0.4, 3.66, func_param,
              usrfunc); // TODO add fmin e fmax
                        // bTargetClusterSendTaskBlocking(mnewt_parallel, &func_param);
#endif // USE_BISECT

        // Search for max F
        // TODO: can I optimize this?
        varValue f_max = 0;
        for (varFor core = 0; core < func_param->num_cores; core++) {
            varFor index_core = core_index[core];
            // printf("index_core: %d / %d - ", core, index_core);
            // printFloat(x_solution[core]);
            // printf("\n\r");
            // TODO: this part is copied from below part: try to merge them
            tptr->values->ma.og_freq[index_core] = x_solution[core];
            /****************/
            vProcessFrequency(index_core, x_solution[core], tptr);

            if (tptr->values->op.computed_core_frequency[index_core] > f_max) {
                f_max = tptr->values->op.computed_core_frequency[index_core];
            }

        } // for core

        // select Vdd
        // TODO optimize this
        varFor index_voltage = 0;
        while (index_voltage < tptr->config_sys->num_voltage_levels) {
            if (f_max <= tptr->config_sys->voltage_table[index_voltage * 3 + 2]) // TODO here just
                                                                                 // lloking at the
                                                                                 // min freq, not
                                                                                 // considering
                                                                                 // voltage
                                                                                 // overheads or
                                                                                 // other changes
            {
                tptr->values->op.computed_domain_voltage[domain] =
                    tptr->config_sys->voltage_table[index_voltage * 3 + 0];
                // printf("voltage index domain %d = %d\n\r", domain, index_voltage);
                tptr->values->op.store_max_freq[domain] =
                    tptr->config_sys->voltage_table[index_voltage * 3 + 2];
                break; // index_voltage = tptr->config_sys->num_voltage_levels+1; //TBC:
            } else
                index_voltage++;
        }
        // checks
        if (index_voltage == tptr->config_sys->num_voltage_levels) {
            // TODO error > max!

            // TODO: put target at max

            // casting vote:
            tptr->values->op.computed_domain_voltage[domain] =
                tptr->config_sys->voltage_table[(index_voltage - 1) * 3 + 0];
            tptr->values->op.store_max_freq[domain] =
                tptr->config_sys->voltage_table[(index_voltage - 1) * 3 + 2];
        }

    } // for domain

    return 0;
}

varFor lPopulateCouplingStructure(varFor i_start, varFor i_end, varFor i_increment,
                                  struct ctrl_task_index *tptr, void **args)
// varFor i_domain, struct coupling_p2f *i_func_param)
{
    varFor *ptr_cast = (varFor *)args[0];
    varFor i_domain = *ptr_cast;
    struct coupling_p2f *i_func_param = (struct coupling_p2f *)args[1];

    varFor counter_core = 0;

#ifndef USE_BISECT
    // TODO: for simplicity add: FMAX, FMIN, VMAX, VMIN in the SysConfigTable?
    varValue bound_k_max =
        i_func_param->k_vdd_stat *
        tptr->config_sys->voltage_table[(tptr->config_sys->num_voltage_levels - 1) * 3] *
        tptr->config_sys->voltage_table[(tptr->config_sys->num_voltage_levels - 1) * 3];
    varValue bound_k_min = i_func_param->k_vdd_stat * tptr->config_sys->voltage_table[0] *
                           tptr->config_sys->voltage_table[0];
    varValue bound_fv_max =
        tptr->config_sys->voltage_table[(tptr->config_sys->num_voltage_levels - 1) * 3] *
        tptr->config_sys->voltage_table[(tptr->config_sys->num_voltage_levels - 1) * 3] *
        tptr->config_sys->voltage_table[(tptr->config_sys->num_voltage_levels - 1) * 3 + 2];
    varValue bound_fv_min = tptr->config_sys->voltage_table[0] *
                            tptr->config_sys->voltage_table[0] *
                            tptr->config_sys->voltage_table[0 + 1];
#endif // not USE_BISECT

    for (varFor core = i_start; core < i_end; core += i_increment) {
        if (tptr->config_sys->core_domain_id[core] == i_domain) {
            i_func_param->ptr_wl[counter_core] = tptr->values->ip.workload[core];
            i_func_param->ptr_Pc[counter_core] = tptr->values->target_core_power[core];
            // TODO: here also put the option of precedent computed frequency
            // i_func_param->ptr_x[counter_core]  = tptr->inputs->cmd.target_freq[core];
            // TODO here also check for Turboing
            // TBD: here tptr->values->core_frequency or tptr->inputs->cmd.target_freq?
            i_func_param->ptr_x[counter_core] =
                (tptr->values->op.computed_core_frequency[core] < tptr->values->core_frequency[core]
                     ? tptr->values->op.computed_core_frequency[core]
                     : tptr->values->core_frequency[core]);
            i_func_param->ptr_cindex[counter_core] = core;

            // TODO improve this, also put this as "minimum accepted performance"
            i_func_param->ptr_lim_inf[counter_core] = 0.4;
            // TODO here also check for Turboing
            // TBD: here tptr->values->core_frequency or tptr->inputs->cmd.target_freq?
            i_func_param->ptr_lim_sup[counter_core] = tptr->inputs->cmd.target_freq[core];

// Test Boundaries:
#ifndef USE_BISECT
            if ((i_func_param->ptr_Pc[counter_core] - i_func_param->k_stat - bound_k_max) /
                    i_func_param->ptr_wl[counter_core] >
                bound_fv_max) // TBC: is it bound_k_max or bound_k_min?
            {
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("NR bound issue, max\r\n");
#endif
                varValue future_Pc = i_func_param->ptr_wl[counter_core] * bound_fv_max +
                                     bound_k_max + i_func_param->k_stat;
                // ifdef check on number
                if (future_Pc >= i_func_param->ptr_Pc[counter_core]) {
                    l_error_map |= BM_ERROR_NUMBERS_VALUE;
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                    printf("Number issue, future_Pc max: ");
                    printFloat(future_Pc);
                    printf("\n\r");
#endif

                    // TODO: something
                }

                // func_param_Pc[counter_core] += func_param_Pc[counter_core] - future_Pc; ADD IN
                // THE BIN, not HERE!
                i_func_param->ptr_Pc[counter_core] = future_Pc;
            } else if ((i_func_param->ptr_Pc[counter_core] - i_func_param->k_stat - bound_k_min) /
                           i_func_param->ptr_wl[counter_core] <
                       bound_fv_min) // TBC: is it bound_k_max or bound_k_min?
            {
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                printf("NR bound issue, min. Pc, Total: ");
                printFloat(i_func_param->ptr_Pc[counter_core]);
                printf(" , ");
                printFloat(
                    (i_func_param->ptr_Pc[counter_core] - i_func_param->k_stat - bound_k_min) /
                    i_func_param->ptr_wl[counter_core]);
                printf("\n\r");
#endif

                varValue future_Pc = i_func_param->ptr_wl[counter_core] * bound_fv_min +
                                     bound_k_min + i_func_param->k_stat;
                // ifdef check on number
                if (future_Pc <= i_func_param->ptr_Pc[counter_core]) {
                    l_error_map |= BM_ERROR_NUMBERS_VALUE;
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                    printf("Number issue, future_Pc min: ");
                    printFloat(future_Pc);
                    printf("\n\r");
#endif

                    // TODO: something
                }

                // func_param_Pc[counter_core] += func_param_Pc[counter_core] - future_Pc; //HERE
                // NEGATIVE!, BC! ADD IN THE BIN, not here!
                i_func_param->ptr_Pc[counter_core] = future_Pc;
            }
#endif // not USE_BISECT

            counter_core++;
        }
        /*
        else
        {
            func_param_wl[core] = 0;
            func_param_Pc[core] = 0;
            x_solution[core] 		= 0;
        }
        */
    }

    return counter_core;
}

void vProcessFrequency(varFor i_core, varValue i_frequency, struct ctrl_task_index *tptr) {
    // Naif Way to smooth the Frequency increase
    // TODO: improve this

    if (i_frequency > tptr->values->op.computed_core_frequency[i_core] +
                          tptr->config_ctrl->output.freq_max_gradient) {
        tptr->values->op.computed_core_frequency[i_core] +=
            tptr->config_ctrl->output.freq_max_gradient;
        tptr->values->freq_red_map[i_core] |= BM_FRM_MAX_GRADIENT;
    } else {
        tptr->values->op.computed_core_frequency[i_core] = i_frequency;
    }

    // TODO: can I optimize this formula? Because I have to pass all the time the address of the
    // Formula Coeff which are fixed per 2 iteration of the control task

    // Hysteresis Check for the temperature going higher than threshold
    // ATTENTION!!! Not thought values can make the PID not to work!
    if ((tptr->inputs->msr.temp.core[i_core] >= tptr->config_ctrl->therm.hyst_high_temp_limit) ||
        (tptr->values->th.hyst_thresh_reached[i_core])) {
        if (tptr->inputs->msr.temp.core[i_core] < tptr->config_ctrl->therm.hyst_low_temp_limit) {
            tptr->values->th.hyst_thresh_reached[i_core] = PCF_FALSE;
        } else {
            tptr->values->th.hyst_thresh_reached[i_core] = PCF_TRUE;
            tptr->values->op.computed_core_frequency[i_core] =
                tptr->config_sys->core_min_frequency; // TBD:

            tptr->values->freq_red_map[i_core] |= BM_FRM_HYST_EMERG;
        }
    }

    if (tptr->values->op.computed_core_frequency[i_core] < tptr->config_sys->core_min_frequency) {
// TBC: is this really an error value?
// TODO c_error_map |= BM_ERROR_NUMBERS_VALUE;
// printf("Number issue, computed_core_frequency, core %d: %d\n\r", i,
// (int)(computed_core_frequency[i_core]*1000.0f));
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
        // printf("Number issue, computed_core_frequency, core %d: %f\n\r", i,
        // computed_core_frequency[i_core]);
        printf("Number issue, computed_core_frequency, core %d: ", i_core);
        printFloat(tptr->values->op.computed_core_frequency[i_core]);
        printf("\n\r");
#endif

        tptr->values->op.computed_core_frequency[i_core] = tptr->config_sys->core_min_frequency;

    } else if (tptr->values->op.computed_core_frequency[i_core] >
               tptr->config_sys->core_max_frequency) {
// TBC: is this really an error value?
// TODO c_error_map |= BM_ERROR_NUMBERS_VALUE;
// printf("Number issue, computed_core_frequency, core %d: %d\n\r", i,
// (int)(computed_core_frequency[i_core]*1000.0f));
#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
        // printf("Number issue, computed_core_frequency, core %d: %f\n\r", i,
        // computed_core_frequency[i_core]);
        printf("Number issue, computed_core_frequency, core %d: ", i_core);
        printFloat(tptr->values->op.computed_core_frequency[i_core]);
        printf("\n\r");
#endif

        tptr->values->op.computed_core_frequency[i_core] = tptr->config_sys->core_max_frequency;

        // for the power filtering
        // pw_filter_removed_freq[i_core] += (target_single_frequency*VD_100_PERC) /
        // (computed_core_frequency[i_core]*VD_100_PERC);
    }
}

varValue fFrequency2PowerConversion(varFor i_start, varFor i_end, varFor i_increment,
                                    struct ctrl_task_index *tptr, void **args) {
    for (varFor i = i_start; i < i_end; i += i_increment) {
        varValue power_formula_coeff[PCF_MODEL_COEFF];
        // TODO: improve this
        power_formula_coeff[0] = tptr->config_sys->base_power_coeff[0][0];
        power_formula_coeff[1] = tptr->values->ip.workload[i];

        varValue target_single_frequency = lMathFrequencyCompute(
            tptr->values->target_core_power[i],
            tptr->values->op.computed_domain_voltage[tptr->config_sys->core_domain_id[i]],
            power_formula_coeff);

        tptr->values->ma.og_freq[i] = target_single_frequency;

        // printf("TF: ");
        // printFloat(target_single_frequency);
        // printf("\n\r");

        /*
        //TBD MOVED HERE FROM BELOW
        // not-proper-analisys: it is better below
        //CUGV //TODO move to a better place.
        varValue alpha = tptr->config_ctrl->freq_diff_alpha;

        //update freq_diff_red paramenter:
        //Moving average to avoid oscillation
        //if instant instead, just use alpha = 1
        c_parameters_table.freq_diff_red[i] *= (VD_ONE - alpha);
        c_parameters_table.freq_diff_red[i] += (c_parameters_table.target_freq[i] -
        target_single_frequency) * alpha;
        */

        /*
        if (i==0){
        printf("Fc: ");
        printFloat(target_single_frequency);
        printf("\n\r");}
        */

        vProcessFrequency(i, target_single_frequency, tptr);
    }

    return 0;
}

void vTelemetryManagement(void /*struct ctrl_task_index* tptr*/) {
    /*** B.2: Write Accumulation Values ***/
    // Check if the Window Horizon for the measurement is reached
    // TBD: Better to do 2 for(36) and 1 if, or 1 for and 36if? If the second is better, move this
    // if inside the for above.

    /*

    if (p_values_ctrl->telemetry_counter <= 0) //TBD: <= or == ? <= also take into account some
    errors.
    {
            p_values_ctrl->telemetry_counter = tptr->inputs->cmd.telemetry_horizon;

            c_telemetry_table.chip_est_pw = 0;
    //TODO: Optimize this for parallel
            for (varFor i = 0; i < tptr->config_sys->num_cores; i++)
            {
                    //TBD: it is better to have a new variable for (varValue)TelemetryHorizion or it
    is ok like this?
                    c_telemetry_table.avg_core[i].freq	/=
    (varValue)tptr->inputs->cmd.telemetry_horizon;
                    c_telemetry_table.avg_core[i].voltage 	    /=
    (varValue)tptr->inputs->cmd.telemetry_horizon;
                    c_telemetry_table.avg_core[i].temp  /=
    (varValue)tptr->inputs->cmd.telemetry_horizon;

                    c_telemetry_table.avg_core[i].est_pw 		    /=
    (varValue)tptr->inputs->cmd.telemetry_horizon;
                    c_telemetry_table.chip_est_pw			    +=
    c_telemetry_table.avg_core[i].est_pw;
            }
            //c_telemetry_table.chip_est_pw 				/=
    (varValue)tptr->inputs->cmd.telemetry_horizon;

            //test_on_number
    //TODO
    /*
            if ((c_telemetry_table.chip_avg_estimated_power) >
    (tptr->config_sys->core_max_power_single * (varValue)tptr->config_sys->num_cores) ) //TODO:
    EPI_CORE_IDLE_POWER
            {
                    l_error_map |= BM_ERROR_NUMBERS_VALUE;
                    #if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
                    //printf("Number issue, chip_avg_estimated_power: %f\n\r",
    c_telemetry_table.chip_avg_estimated_power);
                    printf("Number issue, chip_avg_estimated_power: ");
                    printFloat(c_telemetry_table.chip_avg_estimated_power);
                    printf("\n\r");
                    #endif

                    //TODO: something
            }
    */

    // TODO: DATE_PAPER, remove this.

    /*

    bImpWriteFreqRedMap(&c_telemetry_table);

    //if (bWriteGlobalVariable(&c_telemetry_table, G_TELEMETRY, GLOBAL_WRITE_ALL,
sizeof(c_telemetry_table), c_task_id) != PCF_TRUE)
    //{
            //TODO

    //	#if (defined(PRINTF_ACTIVE) && defined(DEBUG_PRINTF))
    //	printf("bWriteGlobalVariable issue: c_telemetry_table, id: %d\n\r", c_task_id);
    //	#endif
    //}

    // Resetting the Accumulation Internal Variables
//TODO: optimize for cluster
    for (varFor i = 0; i < tptr->config_sys->num_cores; i++)
    {
            c_telemetry_table.avg_core[i].freq 	= 0;
            c_telemetry_table.avg_core[i].voltage 		= 0;
            c_telemetry_table.avg_core[i].temp	= 0;

            c_telemetry_table.avg_core[i].est_pw			= 0;

            //TBD: the FRM tells the SUM of the story or just the last iteration??
            c_telemetry_table.frequency_reduction_map[i]			= BM_RESET;
    }
    c_telemetry_table.chip_est_pw					= 0;
}

/*** Not sure the right position for this code, it may be put while waiting for the DMA ***/
    // TBD: we don't need to do it every cycle? only when bmc ask? Do I or everyhing?

    // write telemetry
}
