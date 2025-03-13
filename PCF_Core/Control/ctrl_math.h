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

//TODO:
/*
* Used https://github.com/pmineiro/fastapprox/tree/master/fastapprox/src
*
*
*/


#ifndef _CTRL_MATH_H_
#define _CTRL_MATH_H_

//#include "FreeRTOS_util.h"
#include "cfg_types.h"
#include "cfg_firmware.h"

//TODO delete //TBU
#include <stdint.h>

#ifdef JO_TBR
#if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
#define V_Fixed                             0.75f
#else
#define V_Fixed                             1
#endif
#endif

/* Functions Declaration */
varValue lMathPidCompute(varValue i_target_power, varValue i_measured_temperature, varFor i_core_number, struct ctrl_thermal* th, ctrl_config_table_t* config_ctrl);
varBool_e bMathPidSetParameters(struct ctrl_thermal* therm_table);

varValue lMathPowerCompute(varValue i_target_frequency, varValue i_target_voltage, varValue* i_formula_coeff);
varValue lMathFrequencyCompute(varValue i_core_target_power, varValue i_domain_target_voltage, varValue* i_formula_coeff) ;
//pid_parameters_t TablePIDParameters; //TBD: to create a struc for PID parameters

uint32_t lMathFindGCD(uint32_t a, uint32_t b);
uint32_t lMathFindArrayGCD(uint32_t* array, int dim);

varValue lMathLmsRecursive(varValue *o_param, varValue *o_Pcurr, varValue *i_prev_param, varValue *i_Pprev, varValue *i_input_value, varValue i_error, varValue i_lambda);
void vMathLmsStoreP(varValue *o_param, varValue *i_input_value);

void MatMul(varValue *O, varValue *iA, varValue *iB, uint16_t r1, uint16_t col1, uint16_t col2);

///

//TODO
//#ifdef USE_NEWTON_RAPSON
struct coupling_p2f {
	varValue *ptr_wl;
	varValue *ptr_Pc;
    varValue *ptr_x;
    varFor *ptr_cindex;
    varValue *ptr_target;
    varValue *ptr_lim_inf;
    varValue *ptr_lim_sup;
	varFor num_cores;
	varValue alpha_vdd;
    varValue aoffset_vdd;
	varValue k_vdd_stat;
	varValue k_stat;
    //
    varFor ntrial;
    varValue tolx; 
	varValue tolf;
    void (*usrfun)(float *,int ,float *,float(*)[PCF_CORES_MAX], void*);
};

//TODO
//#ifdef USE_NEWTON_RAPSON
void pwrfunc(float *F, int n, float *f, float(*J)[PCF_CORES_MAX], void* p);
void usrfunc(float *F, int n, float *f, float(*J)[PCF_CORES_MAX], void* p);

void ludcmp(float (*a)[PCF_CORES_MAX], int n, int *indx, float *d);
void lubksb(float (*a)[PCF_CORES_MAX], int n, int *indx, float b[]);
//void inverse_mat (float (*y)[PCF_CORES_MAX], float (*a)[PCF_CORES_MAX], int N);
//float mat_det(float (*a)[PCF_CORES_MAX], int N);
void mnewt(int ntrial, float x[], int n, float tolx, float tolf, float lim_inf, float lim_sup, void *param, void (*usrfun)(float *,int ,float *,float(*)[PCF_CORES_MAX], void*));

void bisec(int ntrial, varValue x[], int n, varValue tolx, varValue tolf, varValue* lim_inf, varValue* lim_sup, void *param,
    void (*usrfun)(varValue *,int ,varValue *,varValue(*)[PCF_CORES_MAX], void*));

typedef struct {
    float *__restrict__ pSrc;
    float *__restrict__ pDst;
    uint32_t *__restrict__ flag;
    uint32_t N;
    uint32_t nPE;
} plp_mat_inv_instance_f32;

typedef struct {
    const float *__restrict__ pSrcA;
    const float *__restrict__ pSrcB;
    uint32_t M;
    uint32_t N;
    uint32_t O;
    uint32_t nPE;
    float *__restrict__ pDstC;
} plp_mat_mult_instance_f32;

void plp_mat_mult_f32p_xpulpv2(void *args);
int plp_mat_inv_f32p_xpulpv2(void *args);

varValue der_softmax_square(varValue *x, varValue xj, varValue alpha, varFor n);
varValue softmax_square(varValue *x, varValue alpha, varFor n);
varValue softmax(varValue *x, varValue alpha, varFor n);
float fastexp (float p);
float fastpow2 (float p);
void usrfunc(float *F, int n, float *f, float(*J)[PCF_CORES_MAX], void* p);

varValue alpha_softmax;

/*
struct nwrp_parallel {
	int ntrial;
	float* x;
	int n; 
	float tolx; 
	float tolf; 
	void *param;
	void (*usrfun)(float *,int ,float *,float(*)[PCF_CORES_MAX], void*);
};
*/

void mnewt_parallel(void* args);

//#endif //USE_NEWTON_RAPSON

#endif
