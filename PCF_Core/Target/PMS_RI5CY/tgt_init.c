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

#include "tgt_init.h"

/* Target Includes */
#include "system.h"

/* Libraries Inclusion */
#include "cfg_types.h"

varBool_e bTargetSetVoltage(int iVoltage) {

    // Nothing to do here
    return PCF_TRUE;
}
varBool_e bTargetSetFrequency(int iFrequency) {
    // Nothing to do here
    return PCF_TRUE;
}
int lTargetInitializeMemory(void) {
    // Nothing to do here
    return 0;
}
varBool_e bTargetInitVoltage(void) {
    // Nothing to do here
    return PCF_TRUE;
}
varBool_e bTargetInitFrequency(void) {
    // Nothing to do here
    return PCF_TRUE;
}
varBool_e bTargetInitHw(void) {

    system_init();

    return PCF_TRUE;
}

varBool_e bTargetSetTimer(uint32_t time_us) {
    varBool_e return_value = PCF_TRUE;

    // NVIC_SetVector( FC_EVENT_TIMER1, (uint32_t)__handler_wrapper_light_TIMER1_IRQ_handler);
    // NVIC_EnableIRQ( FC_EVENT_TIMER1 );
    // RTDO: Create APIs to set Vector Interrupt

    return return_value;
}

varBool_e bTargetStartTimer(void) {
    varBool_e return_value = PCF_TRUE;

    // RTDO: Start Timer

    return return_value;
}
