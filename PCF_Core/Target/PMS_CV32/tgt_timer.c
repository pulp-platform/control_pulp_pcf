
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


/********************************************************/
/*
* File:
* Notes:
*
* Written by: Eventine (UNIBO)
*
*********************************************************/

#include "tgt_timer.h"

/* Target Includes */
#include "system.h"
#include "io.h"
#include "irq.h"
#include "memory_map.h"
#include "timer.h"
#include "timer_hal.h"
#include "timer_irq.h"

/* Libraries Inclusion */
#include "cfg_types.h"
#include "cfg_constants.h"

#ifdef PRINTF_ACTIVE
#include <stdio.h>
#include "print_float.h"
#endif


#define TIMER_FREQ_HZ           32000
#define MIN_TIMER_US            (CONST_US_IN_A_SEC / TIMER_FREQ_HZ)

// Use TIMER1_HI
timer_id_t timer_id = TIMER_HI_ID;

varBool_e bTargetSetTimer (uint32_t time_us)
{
    varBool_e return_value = PCF_TRUE;

    /* hook up timer1 interrupt */
    /*  Timers in CV32: Timer0 = Systick, Timer1 = free2use
    *   irq_set_handler()
    *   * timer
    *   * void handler(void)
    */
    irq_set_handler(FC_TIMER0_HI_IRQN, TIMER1_IRQ_handler);

    /* reset timer (not really necessary in this case) */
    reset_timer_fc(timer_id);

    /* set interrupt frequency to TIMER1_HI_TICK_RATE_HZ */
    uint32_t timer_freq = CONST_US_IN_A_SEC / time_us;
    set_timer_irq_freq(ARCHI_REF_CLOCK/timer_freq, timer_id);

    //printf("Timer config: ref_clock: %d  timer_freq: %d, tot: %d\n", ARCHI_REF_CLOCK, timer_freq, ARCHI_REF_CLOCK / timer_freq);

    return return_value;
}

varBool_e bTargetStartTimer(void)
{
    varBool_e return_value = PCF_TRUE;

    /* Enable timer (TIMER_CFG_HI_ENABLE_MASK), use 32khz ref clock as
     * source (TIMER_CFG_HI_CLKCFG_MASK). Timer will reset automatically
     * (TIMER_CFG_HI_MODE_MASK) to zero after causing an interrupt
     * (TIMER_CFG_HI_IRQEN_MASK). Also reset timer to start from a clean
     * slate (TIMER_CFG_HI_RESET_MASK).
     */

    /* configure the timer to reset after the configured tick, and trigger an interrupt */
    unsigned int timer_cfg = TIMER_CFG_HI_ENABLE_MASK | TIMER_CFG_HI_RESET_MASK |
    TIMER_CFG_HI_CLKCFG_MASK | TIMER_CFG_HI_MODE_MASK |
    TIMER_CFG_HI_IRQEN_MASK;

    set_timer_cfg(timer_cfg, timer_id);

    /* Enable timer1 interrupt. Need to enable this in the CV32E40P and the
     * apb_interrupt controller. In RI5CY we didn't need to touch the clint
     * since it was in a kind of "passthrough" mode. */
    irq_enable(IRQ_FC_EVT_TIMER0_HI);
    irq_clint_enable(IRQ_FC_EVT_TIMER0_HI);


    return return_value;
}

varBool_e bTargetStopTimer(void)
{
    varBool_e return_value = PCF_TRUE;

    //TODO: here I disabled only the interrupts.
    irq_clint_disable(IRQ_FC_EVT_TIMER0_HI);
    irq_disable(IRQ_FC_EVT_TIMER0_HI);

    return return_value;
}


uint32_t usTargetTimerTapTest( uint32_t tap_us)
{

    uint32_t applied_tap_us = tap_us;

    uint32_t tap_freq = CONST_US_IN_A_SEC / (uint32_t)tap_us;

    uint32_t div_check = TIMER_FREQ_HZ / tap_freq;

    if ( (div_check * tap_freq) != TIMER_FREQ_HZ )
    {
        //ifNdef because check is frequency multiplier: higher is faster/shorter
        #ifndef TAP_INTERVAL_APPROXIMATE_CEIL
        div_check += 1;
        #endif

        #ifdef PRINTF_ACTIVE
        printf("Issue! %dus could not be applied, %dus will applied instead.\n", tap_us, (CONST_US_IN_A_SEC / (TIMER_FREQ_HZ / div_check)) );
        #endif

        //TODO: something.

    }

    applied_tap_us = CONST_US_IN_A_SEC / (TIMER_FREQ_HZ / div_check);

    if (applied_tap_us < MIN_TIMER_US)
    {

        #ifdef PRINTF_ACTIVE
        printf("Issue! Tap is too low. Reset to %d\n", MIN_TIMER_US);
        #endif

        //TODO: something.
        applied_tap_us = (uint32_t)MIN_TIMER_US; //TODO, put default!
    }

    return applied_tap_us;
}

