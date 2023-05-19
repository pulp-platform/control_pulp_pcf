
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


/***************************************************/
/*
* Library to perform the measurement for the project,
* Using FreeRTOS
*
* Date of creation: 31/05/2019
*
* Date of Last Modification: 31/05/2019
*
*/

#include "measure.h"

#include "cfg_firmware.h"
//#include "gap_common.h"

/*** REG Register Definitions ***/
//These are here instead of in the incude.h because they are Taget-dependent.

#define REG_CYCLE_Pos                 0U                                            /*!< REG: CYCLE Position */
#define REG_CYCLE_Msk                 (0x1UL /*<< REG_CYCLE_Pos*/)                 /*!< REG: CYCLE Mask */

#define REG_INSTR_Pos                 1U                                            /*!< REG: _INSTR Position */
#define REG_INSTR_Msk                 (0x1UL << REG_INSTR_Pos)                     /*!< REG: _INSTR Mask */

#define REG_LD_STALL_Pos              2U                                            /*!< REG: LD_STALL Position */
#define REG_LD_STALL_Msk              (0x1UL << REG_LD_STALL_Pos)                  /*!< REG: LD_STALL Mask */

#define REG_JMP_STALL_Pos             3U                                            /*!< REG: JMP_STALL Position */
#define REG_JMP_STALL_Msk             (0x1UL << REG_JMP_STALL_Pos)                 /*!< REG: JMP_STALL Mask */

#define REG_IMISS_Pos                 4U                                            /*!< REG: IMISS Position */
#define REG_IMISS_Msk                 (0x1UL << REG_IMISS_Pos)                     /*!< REG: IMISS Mask */

#define REG_WBRANCH_Pos               5U                                            /*!< REG: WBRANCH Position */
#define REG_WBRANCH_Msk               (0x1UL << REG_WBRANCH_Pos)                   /*!< REG: WBRANCH Mask */

#define REG_WBRANCH_CYC_Pos           6U                                            /*!< REG: WBRANCH_CYC Position */
#define REG_WBRANCH_CYC_Msk           (0x1UL << REG_WBRANCH_CYC_Pos)               /*!< REG: WBRANCH_CYC Mask */

#define REG_LD_Pos                    7U                                            /*!< REG: LD Position */
#define REG_LD_Msk                    (0x1UL << REG_LD_Pos)                        /*!< REG: LD Mask */

#define REG_ST_Pos                    8U                                            /*!< REG: ST Position */
#define REG_ST_Msk                    (0x1UL << REG_ST_Pos)                        /*!< REG: ST Mask */

#define REG_JUMP_Pos                  9U                                            /*!< REG: JUMP Position */
#define REG_JUMP_Msk                  (0x1UL << REG_JUMP_Pos)                      /*!< REG: JUMP Mask */

#define REG_BRANCH_Pos                10U                                           /*!< REG: BRANCH Position */
#define REG_BRANCH_Msk                (0x1UL << REG_BRANCH_Pos)                    /*!< REG: BRANCH Mask */

#define REG_DELAY_SLOT_Pos            11U                                           /*!< REG: DELAY_SLOT Position */
#define REG_DELAY_SLOT_Msk            (0x1UL << REG_DELAY_SLOT_Pos)                /*!< REG: DELAY_SLOT Mask */

#define REG_LD_EXT_Pos                12U                                           /*!< REG: LD_EXT Position */
#define REG_LD_EXT_Msk                (0x1UL << REG_LD_EXT_Pos)                    /*!< REG: LD_EXT Mask */

#define REG_ST_EXT_Pos                13U                                           /*!< REG: ST_EXT Position */
#define REG_ST_EXT_Msk                (0x1UL << REG_ST_EXT_Pos)                    /*!< REG: ST_EXT Mask */

#define REG_LD_EXT_CYC_Pos            14U                                           /*!< REG: LD_EXT_CYC Position */
#define REG_LD_EXT_CYC_Msk            (0x1UL << REG_LD_EXT_CYC_Pos)                /*!< REG: LD_EXT_CYC Mask */

#define REG_ST_EXT_CYC_Pos            15U                                           /*!< REG: ST_EXT_CYC Position */
#define REG_ST_EXT_CYC_Msk            (0x1UL << REG_ST_EXT_CYC_Pos)                /*!< REG: ST_EXT_CYC Mask */

#define REG_TCDM_CONT_Pos             16U                                           /*!< REG: TCDM_CONT Position */
#define REG_TCDM_CONT_Msk             (0x1UL << REG_TCDM_CONT_Pos)                 /*!< REG: TCDM_CONT Mask */

#define REG_EVENTS_NUM                17U                                           /*!< REG: All events number */

#define PCMR_GLBEN_Pos                 0U
#define PCMR_SATU_Pos                  1U

// Addresses mapping

#define CSR_REGCOUNTER(N)       (0x780 + (N))
#define CSR_REGEVENT(M)         (0xCC0 + (M))

// Define asm volatile macros
#define CSR_CONVERT(x)    #x
#define CSR_WRITE(reg, var)  asm volatile ("csrw "CSR_CONVERT(x)", %0" :: "r" (var))
#define CSR_READ(reg, var)   asm volatile ("csrr %0, "CSR_CONVERT(reg)"" : "=r" (var) :)

/*** End ***/

#if (MEASURE_ACTIVE == 1)
int lPerformanceCheck = 0;
int CSCheck = 0;
int Timerindex = 0;
Timer_Data_t timerBuffer[MEASURE_N_OUTPUTS*MEASURE_N_ITERATION+7] = { {'0', 0} };
#endif
/*******************************/
#if (MEASURE_ACTIVE == 2)
uint32_t g_I_cycle_prev = 0;
uint32_t g_cycle_valid_comparison = 0;
uint32_t g_mes_comp_value = 1;
#endif


/***********************/
/*****************************************/
/***********************/
void vMeasureInit ( measure_type_e type )
{
    uint32_t maskConfig = 0x0;

    /* Reset all to 0 */
    asm volatile ("csrw 0x79F, %0" :: "r" (0));

    /** Configure **/
    if ((type & mCycles) == mCycles)
    {
        maskConfig |= (REG_CYCLE_Msk);
    }
    if ((type & mInstr) == mInstr)
    {
        maskConfig |= (REG_INSTR_Msk);
    }
    if ((type & mLoad) == mLoad)
    {
        maskConfig |= (REG_LD_Msk);
    }
    if ((type & mStore) == mStore)
    {
        maskConfig |= (REG_ST_Msk);
    }
    if ((type & mJump) == mJump)
    {
        maskConfig |= (REG_JUMP_Msk);
    }
    if ((type & mBranch) == mBranch)
    {
        maskConfig |= (REG_BRANCH_Msk);
    }
    if ((type & mCInstr) == mCInstr)
    {
        //Nothing
    }

    //

    if ((type & mLdStall) == mLdStall)
    {
        maskConfig |= (REG_LD_STALL_Msk);
    }
    if ((type & mStStall) == mStStall)
    {
        //nothing
    }
    if ((type & mInstrMiss) == mInstrMiss)
    {
        maskConfig |= (REG_IMISS_Msk);
    }
    if ((type & mJmpStall) == mJmpStall)
    {
        maskConfig |= (REG_JMP_STALL_Msk);
    }
    if ((type & mBrchTaken) == mBrchTaken)
    {
        //Nothing
    }
    if ((type & mPipeStall) == mPipeStall)
    {
        //Nothing
    }
    if ((type & mApuDependStall) == mApuDependStall)
    {
        //Nothing
    }

    //

    if ((type & mApuTypeConfl) == mApuTypeConfl)
    {
        //Nothing
    }
    if ((type & mApuContention) == mApuContention)
    {
        //Nothing
    }
    if ((type & mApuWriteBack) == mApuWriteBack)
    {
        //Nothing
    }

   /* Enable event: REG register */
   asm volatile ("csrw 0xCC0, %0" : "+r" (maskConfig) );
}

void vMeasureStart (void)
{
    /* Enable counters: PCMR register */
    uint32_t maskEnable = (1 << PCMR_GLBEN_Pos) | (1 << PCMR_SATU_Pos);
    asm volatile ("csrw 0xCC1, %0" :: "r" (maskEnable));
}
void vMeasureStop (void)
{
    //TODO
}

//TODO: need to implement zeroing for all the functions below
inline uint32_t lMeasureReadCsr(int zeroing) // __attribute__((always_inline))
{
    uint32_t result;

    // This function does not work because compiler cannot "convert" the csr_num to a 12-bit Immediate.
    //asm("csrr %0, %1" : "=r"(result) : "I"(csr_num.value));

    CSR_READ(CSR_REGCOUNTER(MEASURE_CSR), result);


    return result;
}

inline uint32_t lMeasureReadCycle ( int zeroing )
{
    /* Read */
    uint32_t value = 0;
    /* Cycles */
    asm volatile ("csrr %0, 0x780" : "=r" (value));

    return value;
}
inline uint32_t lMeasureReadInstr ( int zeroing )
{
    /* Read */
    uint32_t value = 0;
    /* Instructions */
    asm volatile ("csrr %0, 0x781" : "=r" (value));

    return value;
}
inline uint32_t lMeasureReadLdStall ( int zeroing )
{
    /* Read */
    uint32_t value = 0;
    /* LD stall */
    asm volatile ("csrr %0, 0x782" : "=r" (value));

    return value;
}
inline uint32_t lMeasureReadInstrMiss ( int zeroing )
{
    /* Read */
    uint32_t value = 0;
    /* Istr Stall */
    asm volatile ("csrr %0, 0x784" : "=r" (value));

    return value;
}
inline uint32_t lMeasureReadLoad ( int zeroing )
{
    /* Read */
    uint32_t value = 0;
    /* LD */
    asm volatile ("csrr %0, 0x785" : "=r" (value));

    return value;
}
inline uint32_t lMeasureReadStore ( int zeroing )
{
    /* Read */
    uint32_t value = 0;
    /* Store */
    asm volatile ("csrr %0, 0x786" : "=r" (value));

    return value;
}
