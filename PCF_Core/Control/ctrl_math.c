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

/* Header include. */
#include "ctrl_math.h"

#include "cfg_types.h"
#include "cfg_system.h"
#include "cfg_control.h"
#include "cfg_firmware.h" //todo remove, just for v_fixed
#include "ctrl_functions.h" //Needed for tptr, but dunno if it is ok to do like this

/* Other Inclusion */
//TODO DELETE
#include <stdint.h>

//DELETEEEE
#ifdef PCF_USE_CLUSTER
#include "tgt_cluster.h"
//#include "tgt_cluster_hw_def.h"
#include "cl_team.h"
#include "target.h"

#include "timer.h"
#include "timer_hal.h"
#include "timer_irq.h"

#include "cluster/fc_to_cl_delegate.h"
#include "cluster/event_unit.h"
#include "device.h"
#include "os.h"
#include "cluster/cl_idma_hal.h"
#endif
//
#ifdef PRINTF_ACTIVE
#include <stdlib.h>
#include <stdio.h>
#include "print_float.h"
#endif
#ifdef MEASURE_ACTIVE
#include "tgt_dbg_measure.h"
#endif

//TOCANC
#include <math.h>
//TOCANC
#ifdef GSL_PARALLEL
#include "tgt_cluster.h"
//#include "tgt_cluster_hw_def.h"
#include "cl_team.h"
#include "target.h"

#include "cluster/fc_to_cl_delegate.h"
#include "cluster/event_unit.h"
#include "device.h"
#include "os.h"
#include "cluster/cl_idma_hal.h"

L1_DATA float x_init[PCF_CORES_MAX];	
L1_DATA float func_param_wl[PCF_CORES_MAX];
L1_DATA float func_param_Pc[PCF_CORES_MAX];
#endif

varValue lMathPidCompute(varValue i_target_power, varValue i_measured_temperature, varFor i_core_number, struct ctrl_thermal* th, ctrl_config_table_t* config_ctrl) {

	varValue output_pid = 0;

	/* Compute Error */
	varValue error = th->Tcrit_pid - i_measured_temperature;
	//test_on_number
	//if ( (abs(error) - th->Tcrit_pid) > 0)
	//{
		/* TODO:
		#ifdef ERROR_MAP
		ErrorMap |= BM_ERROR_NUMBERS_VALUE;
		#endif
		*/

		//error = ?? //TODO
	//}

    //pid con deadzone
    /*
    if ( (error<0?-error:error) < 0.5)
    {
        error = 0;
        //Hybrid
        //th->pid_integral_error[i_core_number] = 0;
    }
    */
    if ( (error < 2.5) && (error > 0) ) //(error>0)
    {
        error *= 0.7;
        //error = _pid_previous_error[i_core_number];
        //Hybrid
        //th->pid_integral_error[i_core_number] = 0;
    }
    if ( (error > -1.0) && (error < 0) ) //(0)
    {
        //error =  error*(-0.1794 + error*(1.2388 + error*(-1.9916 + error*1.11265)));
        error *= 0.7;
	//error = 0;
        //error = _pid_previous_error[i_core_number];

        //Hybrid
        //th->pid_integral_error[i_core_number] = 0;
    }

	//printf("er: ");
	//printFloat(error);
	/* Proportional Term */
	output_pid = th->kp * error;
/*
	if ((i_measured_temperature> EPI_CORE_CRITICAL_TEMPERATURE - 50) && (i_core_number == 0))
	{
		printf("asd  ");
		printFloat(output_pid);
	}*/

	/* Integral Term */
	if (config_ctrl->conf.use_pid_integral_action == PCF_TRUE)
	{
		//TODO:
		// switch (iComm)
		// {
		// 	case 1:
		// 		th->pid_integral_error[i_core_number] *= 0.5;
		// 		break;
		// 	case 2:
		// 		th->pid_integral_error[i_core_number] *= 1.5;
		// 		break;
		// 	default:
		// 		break;
		// }
		/*
		if (i_integr_red != 0)
		{
			th->pid_integral_error[i_core_number] = 0;
			//th->pid_integral_error[i_core_number] =* i_integr_red * PID_INT_RED_COEFF;
		}
		*/
		//if (pbc == PCF_TRUE)
		//	th->pid_integral_error[i_core_number] *= 0.1;
		th->pid_integral_error[i_core_number] += error * th->ki;

		//TODO: this saturation is problematic because it depends on the Target power.
		if (config_ctrl->conf.use_pid_anti_windup_sat == PCF_TRUE)
		{
			if (th->pid_integral_error[i_core_number] > config_ctrl->therm.pid_anti_windup_sat_up ) //TBD: Should I parametrize this 0?
				th->pid_integral_error[i_core_number] = config_ctrl->therm.pid_anti_windup_sat_up;
			else if (th->pid_integral_error[i_core_number] < (config_ctrl->therm.pid_anti_windup_sat_coeff * i_target_power)) //TODO: var conversion
			{
				th->pid_integral_error[i_core_number] = config_ctrl->therm.pid_anti_windup_sat_coeff * i_target_power; //TODO: var conversion
			}
		}
		output_pid += th->pid_integral_error[i_core_number];
	}

	/* Derivative Term */
	if (config_ctrl->conf.use_pid_derivative_action == PCF_TRUE)
	{
		output_pid += th->kd * (error - th->pid_previous_error[i_core_number]);
	}

    /* Update Error Global Variable */
	th->pid_previous_error[i_core_number] = error;

/*
	if ((i_measured_temperature> EPI_CORE_CRITICAL_TEMPERATURE - 6.2) && (i_core_number == 0))
	{
		printf("asd  ");
		printFloat(i_target_power);
		printFloat(output_pid);
	}
*/
//_error_accum[i_core_number] *= (1.0f-0.05f);
//_error_accum[i_core_number] += 0.05f*_kp*error;
//output_pid = _error_accum[i_core_number];
	/* Saturation */
	if (config_ctrl->conf.use_pid_saturation == PCF_TRUE)
	{
		if (output_pid > th->saturation_max)
			output_pid = th->saturation_max;
		else if (output_pid < (-(i_target_power - th->Pidle)) )
			output_pid = -(i_target_power - th->Pidle);
	}

	//printf("outpid: %d, itargetpow: %d\n\r", (int)output_pid, (int)i_target_power);
    //if (!i_core_number){
    //           printf("P: ");printFloat(output_pid);printf("\n\r");}

	//pid ff action
		output_pid = i_target_power + output_pid;

	/* Do this directly is the main to avoid also var assage bug
	#ifdef USE_TESTS_ON_NUMBERS
	if ( (output_pid <= 0) || (output_pid > i_target_power) )
	{
		#ifdef ERROR_MAP
		ErrorMap |= BM_ERROR_NUMBERS_VALUE;
		#endif

		output_pid = EPI_CORE_IDLE_POWER; //TBD: maybe too much conservative in case > ?? (consider still there is an error!)
	}
	#endif
	*/
	 
	return output_pid;
}

varBool_e bMathPidSetParameters(struct ctrl_thermal* therm_table){ //TBD: better to pass by copy or by reference?


	therm_table->kp = g_ControlConfigTable.therm.pid_kp;
	therm_table->ki = g_ControlConfigTable.therm.pid_ki*g_ControlConfigTable.therm.pid_dt;
	therm_table->kd = g_ControlConfigTable.therm.pid_kd*g_ControlConfigTable.therm.pid_dt;

	therm_table->Tcrit_pid = g_config_sys.core_critical_temperature - g_ControlConfigTable.therm.pid_temp_margin;
	therm_table->saturation_max = 0; //TODO
	therm_table->Pidle = g_config_sys.core_idle_power;

	//TODO: check, if everything is ok then
	return PCF_TRUE;// TBD: void or boolean?
}

varValue lMathPowerCompute(varValue i_target_frequency, varValue i_target_voltage, varValue* i_formula_coeff){ //TBD: better to pass by copy or by reference? (same for output)

	/* Formula: P = K1*f*V^2 + K2 + (K3 + K4*f)*w^K5 */

	//return (i_formula_coeff[0] * i_target_frequency * (V_Fixed*V_Fixed) ) + i_formula_coeff[1] +
	//		( (i_formula_coeff[2] + (i_formula_coeff[3] * i_target_frequency)) * iWorkload ); //TODO: Formula miss k5
			//Same formula as Christian. checked.

	//printf("pc: " );
	//printFloat((Icc + (iWorkload * (i_target_frequency /*/ 1000000000.0*/) * V_Fixed) ) * V_Fixed );
	//return ( (Icc + (iWorkload * (i_target_frequency /*/ 1000000000.0*/) * V_Fixed) ) * V_Fixed );

	//printFloat(i_formula_coeff[0]);
	//printFloat(i_formula_coeff[1]);
	//printFloat((i_formula_coeff[0] + (i_formula_coeff[1] * (i_target_frequency /*/ 1000000000.0*/) * V_Fixed) ) * V_Fixed );

	#ifdef USE_RHEA
	return ( (i_formula_coeff[0]*i_target_voltage + (i_formula_coeff[1] * (i_target_frequency /*/ 1000000000.0*/)) ) * i_target_voltage + 0.469f );
	#elif defined(USE_MONTE_CIMONE)
	return ( (i_formula_coeff[0] + (i_formula_coeff[1] / 0.9 / 0.9 * (i_target_frequency /*/ 1000000000.0*/) * i_target_voltage) ) * i_target_voltage + 0.52804f);
	#else //Legacy, HIPEAC
    return ( (i_formula_coeff[0] + (i_formula_coeff[1] * (i_target_frequency /*/ 1000000000.0*/) * i_target_voltage) ) * i_target_voltage );
	#endif

	/* Formula:
	* P = Pd + Ps
	* Pd = Ceff*f*Vdd^2
	* Ceff = [Weights]T * [PerfCounters]
	*/

	//Simulation //TODO: Change Formula
	//Assume Vdd = 10 fixed
	//return i_formula_coeff[0] * i_target_frequency * (10 * 10);

}

varValue lMathFrequencyCompute(varValue i_core_target_power, varValue i_domain_target_voltage, varValue* i_formula_coeff) {

	//TBC: Check formula. Miss (^k5) in the end
	/*return (i_core_target_power - i_formula_coeff[1] - i_formula_coeff[2] * iWorkload) /
								(V_Fixed * V_Fixed * i_formula_coeff[0] + i_formula_coeff[3] * iWorkload);
	*/

	#if ((CONTROL_DATA_TYPE == 1) || (CONTROL_DATA_TYPE == 2))
	//printf("ciaddo\n\r" );
	//return ( ((i_core_target_power - (Icc*V_Fixed)) / iWorkload / (V_Fixed*V_Fixed)) /* *1000000000.0*/);
	varValue step = 10.0f;

	#ifdef USE_RHEA
	varValue frequency = ( ((i_core_target_power - 0.469f) / i_target_voltage) - (i_formula_coeff[0]*i_target_voltage)) / i_formula_coeff[1];
	#elif defined(USE_MONTE_CIMONE)
	varValue frequency = ( (((i_core_target_power - 0.52804f) / i_domain_target_voltage) - i_formula_coeff[0]) / i_domain_target_voltage / i_formula_coeff[1] * 0.9 * 0.9 );
	#else //Legacy, HIPEAC
	varValue frequency = ((i_core_target_power/i_domain_target_voltage - i_formula_coeff[0]) / i_formula_coeff[1] / i_domain_target_voltage);
	#endif
	frequency = (frequency +0.06f) * step;
	uint32_t step_frequency = (uint32_t) frequency;
	#else
	//return ( ((i_core_target_power - (Icc*V_Fixed)) / iWorkload / (V_Fixed*V_Fixed)) /* *1000000000.0*/);
	varValue step = 10;

	//varValue frequency = ( ((i_core_target_power - (i_formula_coeff[0]*V_Fixed)) / i_formula_coeff[1] / (V_Fixed*V_Fixed)));
	varValue frequency = ( ((i_core_target_power - (i_formula_coeff[0]*i_domain_target_voltage)) / i_formula_coeff[1] / (i_domain_target_voltage*i_domain_target_voltage)));
	frequency = (frequency) * step;
	uint32_t step_frequency = (uint32_t) frequency;
	#endif

	return ((varValue)step_frequency / step);
}

uint32_t lMathFindGCD(uint32_t a, uint32_t b)
{
	if (( b < 1) || (a < 0))
		return 0;
	if ( a == 0)
		return b;
	return lMathFindGCD((b % a), a);
}

uint32_t lMathFindArrayGCD(uint32_t* array, int dim)
{
	uint32_t result = array[0];
	varFor i = 0;
	while((result < 1)&&(i < dim-1))
	{
		i++;
		result = array[i];
	}
	if (result < 1)
		return 0;
	for (i = 1; i < dim; i++)
	{
		if (array[i] < 1)
		{
			i++;
			if (i >= dim)
				return result;
		}
		result = lMathFindGCD(array[i], result);

		if (result < 1)
			return 0;
		if (result == 1)
			return 1;
	}

	return result;
}

varValue lMathLmsRecursive(varValue *o_param, varValue *o_Pcurr, varValue *i_prev_param, varValue *i_Pprev, varValue *i_input_value, varValue i_error, varValue i_lambda) {

	// p_est = p_est(prev) + K*error;
	// error = measured_power - power_computed_with_prev_paramenters;
	// k = P*[V, VVF]
	// P = P(prev)/lambda/(t-1) - P(prev)*[V, VVF]*[V, VVF]T*P(prev)/(t-1)/(lambda*(lambda*(t-1) + [V, VVF]T*P(prev)*[V, VVF])))

	varValue T2[LMS_CN];
	varValue T3[LMS_CNxCN];
	varValue T4[LMS_CNxCN];
	varValue K[LMS_CN];
	//float coeff1 = (iPass/(iPass-1))/i_lambda;
	varValue coeff2 = 0;

	MatMul(T2, i_Pprev, i_input_value, LMS_CN, LMS_CN, 1);
	MatMul(T3, T2, i_input_value, LMS_CN, 1, LMS_CN); //since the vector is Nx1, no need to do the Transpose since the Matrix is implemented as an array
	MatMul(T4, T3, i_Pprev, LMS_CN, LMS_CN, LMS_CN);

	// is a number, maybe is worth to create its own function
	//MatMul(coeff2, i_input_value, T2, 1, iN, 1); //since the vector is Nx1, no need to do the Transpose since the Matrix is implemented as an array
	for (int i = 0; i < LMS_CN; i++) //here leave int vs varFor
	{
		coeff2 += i_input_value[i] * T2[i];
	}

	coeff2 = (coeff2 + i_lambda) * i_lambda;

	// MatSum, but optimized!
	const varValue cMATRIX_LIM = 52; //TODO
	for (int i = 0; i < LMS_CNxCN; i++)
	{
		varValue Pcurr_app = ((i_Pprev[i] / i_lambda) - (T4[i] / coeff2));

		//TODO: this limitation are meh
		if (Pcurr_app > cMATRIX_LIM)
			Pcurr_app = cMATRIX_LIM;
		else if (Pcurr_app < -cMATRIX_LIM)
			Pcurr_app = -cMATRIX_LIM;

		o_Pcurr[i] = Pcurr_app;
	}

	MatMul(K, o_Pcurr, i_input_value, LMS_CN, LMS_CN, 1);

	// MatSum, but optimized!
	for (int i = 0; i < LMS_CN; i++)
	{
		o_param[i] = i_prev_param[i] + K[i] * i_error;
	}

	return 0; //here you should return maybe diff between parameters? a value for errors?
}

void vMathLmsStoreP (varValue *o_param, varValue *i_input_value)
{
	varValue T2[PCF_MODEL_COEFF*PCF_MODEL_COEFF];

	MatMul(T2, i_input_value, i_input_value, PCF_MODEL_COEFF, 1, PCF_MODEL_COEFF);

	for (int i=0; i < PCF_MODEL_COEFF*PCF_MODEL_COEFF; i++)
		o_param[i] += T2[i];

	return;
}

void MatMul(varValue *O, varValue *iA, varValue *iB, uint16_t r1, uint16_t col1, uint16_t col2)
{
	uint16_t r2 = col1;

	for (int i = 0; i < r1; i++)
	  for (int j = 0; j < col2; j++)
	  {
		varValue sum = 0.0;
		for (int k = 0; k < r2; k++)
		{
		  sum += iA[i*col1 + k] * iB[k*col2 + j];
		}

		O[i*col2 + j] = sum;
		//sum = 0.0;
	  }

	 return;
}

//TODO
//#ifdef USE_NEWTON_RAPSON

#define TINY 0.0003 //TODO

void ludcmp(float (*a)[PCF_CORES_MAX], int n, int *indx, float *d)
/*
Given a matrix a[1..n][1..n], this routine replaces it by the LU decomposition of a rowwise
permutation of itself. a and n are input. a is output, arranged as in equation (2.3.14) above;
indx[1..n] is an output vector that records the row permutation effected by the partial
pivoting; d is output as +/-1 depending on whether the number of row interchanges was even
or odd, respectively. This routine is used in combination with lubksb to solve linear equations
or invert a matrix.
*/
{
    int i,imax,j,k;
    float big,dum,sum,temp;
    float vv[PCF_CORES_MAX]; //vv stores the implicit scaling of each row. //TODO
    *d=1.0; //No row interchanges yet.
    for (i=0;i<n;i++)  //Loop over rows to get the implicit scaling information.
    {
        big=0.0;
        for (j=0;j<n;j++)
            if ((temp=fabs(a[i][j])) > big) big=temp;
        if (big == 0.0) {printf("Singular matrix in routine ludcmp"); return;}
        //No nonzero largest element.
        vv[i]=1.0/big; //Save the scaling.
    }
    for (j=0;j<n;j++) //This is the loop over columns of Crout's method.
    {
        for (i=0;i<j;i++) //This is equation (2.3.12) except for i = j.
        {
            sum=a[i][j];
            for (k=0;k<i;k++) sum -= a[i][k]*a[k][j];
            a[i][j]=sum;
        }
        big=0.0; //Initialize for the search for largest pivot element.
        for (i=j;i<n;i++) //This is i = j of equation (2.3.12) and i = j + 1 ...N of equation (2.3.13).
        {
            sum=a[i][j]; 
            for (k=0;k<j;k++)
                sum -= a[i][k]*a[k][j];
            a[i][j]=sum;
            if ( (dum=vv[i]*fabs(sum)) >= big) 
            {
                //Is the figure of merit for the pivot better than the best so far?
                big=dum;
                imax=i;
            }
        }
        if (j != imax) //Do we need to interchange rows?
        {
            for (k=0;k<n;k++) //Yes, do so...
            {
                dum=a[imax][k];
                a[imax][k]=a[j][k];
                a[j][k]=dum;
            }
            *d = -(*d); //...and change the parity of d.
            vv[imax]=vv[j]; //Also interchange the scale factor.
        }
        indx[j]=imax;
        if (a[j][j] == 0.0) a[j][j]=TINY;
        //If the pivot element is zero the matrix is singular (at least to the precision of the
        //algorithm). For some applications on singular matrices, it is desirable to substitute
        //TINY for zero.
        if (j != n) //Now, finally, divide by the pivot element.
        {
            dum=1.0/(a[j][j]);
            for (i=j+1;i<n;i++) a[i][j] *= dum;
        }
    } //Go back for the next column in the reduction.

    //free_vector(vv,1,n);
}

void lubksb(float (*a)[PCF_CORES_MAX], int n, int *indx, float b[])
/*
Solves the set of n linear equations AX = B. Here a[1..n][1..n] is input, not as the matrix
A but rather as its LU decomposition, determined by the routine ludcmp. indx[1..n] is input
as the permutation vector returned by ludcmp. b[1..n] is input as the right-hand side vector
B, and returns with the solution vector X . a, n, and indx are not modified by this routine
and can be left in place for successive calls with different right-hand sides b. This routine takes
into account the possibility that b will begin with many zero elements, so it is efficient for use
in matrix inversion.
*/
{
    int i,ii=0,ip,j;
    float sum;
    for (i=0;i<n;i++) 
    {
        //When ii is set to a positive value, it will become the
        //index of the first nonvanishing element of b. We now
        //do the forward substitution, equation (2.3.6). The
        //only new wrinkle is to unscramble the permutation as we go.
        ip=indx[i];
        sum=b[ip];
        b[ip]=b[i];
        if (ii)
            for (j=ii;j<=i;j++) sum -= a[i][j]*b[j];
        else if (sum) ii=i+1; //A nonzero element was encountered, so from now on we will have to do the sums in the loop above.
        b[i]=sum; 
    }
    for (i=n-1;i>=0;i--) //Now we do the backsubstitution, equation (2.3.7).
    {
        sum=b[i];
        for (j=i+1;j<n;j++) sum -= a[i][j]*b[j];
        b[i]=sum/a[i][i]; //Store a component of the solution vector X.
    } //All done!
}

/*
void inverse_mat (float (*y)[PCF_CORES_MAX], float (*a)[PCF_CORES_MAX], int N)
{
    int i,j,*indx;
    float d;
    float col[PCF_CORES_MAX];

    ludcmp(a,N,indx,&d);// Decompose the matrix just once.
    for(j=0;j<N;j++) //Find inverse by columns.
    {
        for(i=0;i<N;i++) col[i]=0.0;
        col[j]=1.0;
        lubksb(a,N,indx,col);
        for(i=0;i<N;i++) y[i][j]=col[i];
    }
}

float mat_det(float (*a)[PCF_CORES_MAX], int N)
{
    int j, *indx;
    float d;
    ludcmp(a,N,indx,&d); //This returns d as +/-1.
    for(j=0;j<N;j++) d *= a[j][j];

    return d;
}
**/
#ifdef GSL_PARALLEL
//TODO
void mnewt_parallel(void* args)
{
	struct nwrp_parallel* casted_args = (struct nwrp_parallel*)args;
	struct coupling_p2f* casted_param = (struct coupling_p2f*) casted_args->param;
	/*L1_DATA*/ struct coupling_p2f L1_param;

	
	//printf("Starting mnewt parallel\r\n");

	unsigned int dma_id_temp = pulp_cl_idma_memcpy(
				   x_init,
				   casted_args->x,
				   sizeof(float)*PCF_CORES_MAX);
	plp_cl_dma_wait(dma_id_temp);
	dma_id_temp = pulp_cl_idma_memcpy(
				   func_param_wl,
				   casted_param->ptr_wl,
				   sizeof(float)*PCF_CORES_MAX);
	plp_cl_dma_wait(dma_id_temp);
	dma_id_temp = pulp_cl_idma_memcpy(
				   func_param_Pc,
				   casted_param->ptr_Pc,
				   sizeof(float)*PCF_CORES_MAX);
	plp_cl_dma_wait(dma_id_temp);

	L1_param.ptr_wl = func_param_wl;
	L1_param.ptr_Pc = func_param_Pc;
	L1_param.num_cores = casted_param->num_cores;
	L1_param.alpha_vdd = casted_param->alpha_vdd;

	//printf("done copying\r\n");

	mnewt(casted_args->ntrial, x_init, casted_args->n, casted_args->tolx, casted_args->tolf, &L1_param, casted_args->usrfun);

	dma_id_temp = pulp_cl_idma_memcpy(
				   casted_args->x,
				   x_init,
				   sizeof(float)*PCF_CORES_MAX);
	plp_cl_dma_wait(dma_id_temp);
}
#endif //GSL_PARALLEL

void mnewt(int ntrial, float x[], int n, float tolx, float tolf, float lim_inf, float lim_sup, void *param, 
    void (*usrfun)(float *,int ,float *,float(*)[PCF_CORES_MAX], void*))
/*
Given an initial guess x[1..n] for a root in n dimensions, take ntrial Newton-Raphson steps
to improve the root. Stop if the root converges in either summed absolute variable increments
tolx or summed absolute function values tolf.
*/
{
    int k,i;
    float errx,errf,d;

    int indx[PCF_CORES_MAX];
    float p[PCF_CORES_MAX];
    float fvec[PCF_CORES_MAX];
    float fjac[PCF_CORES_MAX][PCF_CORES_MAX];

    varValue saved_init[PCF_CORES_MAX];
    for (i=0;i<n;i++)
    {
        saved_init[i]=x[i];
    }


    for (k=0;k<ntrial;k++) {
        usrfun(x,n,fvec,fjac, param); //User function supplies function values at x in fvec and Jacobian matrix in fjac.
        errf=0.0; 
        for (i=0;i<n;i++) errf += fabs(fvec[i]); //Check function convergence.
        if (errf <= tolf) break; //FREERETURN //TODO
        for (i=0;i<n;i++) p[i] = -fvec[i]; //Right-hand side of linear equations.

        //printf("Done userfunc\r\n");
        int diverged = 0;
        varValue divergence_factor = 0;

        #ifndef GSL_PARALLEL
        ludcmp(fjac,n,indx,&d); //Solve linear equations using LU decomposition.
        lubksb(fjac,n,indx,p);
        errx=0.0; //Check root convergence.
        for (i=0;i<n;i++) //Update solution.
        {
            errx += fabs(p[i]);
            x[i] += p[i];
            printf("Xd[%d]: ", i);
            printFloat(x[i]);

           	varValue value1 = x[i] - lim_inf;
        	varValue value2 = x[i] - lim_sup;
        	printf(" - ");
        	printFloat(value1);
        	printf(" - ");
        	printFloat(value2);
    		printf("\n\r");

            if (value1 < 0)
            {
                diverged = 1;
                varValue value1_app = value1;
                if(value1_app <0)
                	value1_app = -value1_app;
                if (value1_app > divergence_factor)
                    divergence_factor = value1;            

            } else if (value2>0) 
            {
                diverged = 1;
                if (value2 > divergence_factor)
    				divergence_factor = value2;
            }
        }
        #else //GSL_PARALLEL
        float inv_fjac[PCF_CORES_MAX][PCF_CORES_MAX];
        struct coupling_p2f *param_casted = (struct coupling_p2f*)param;
        plp_mat_inv_instance_f32 args = {.pSrc = fjac, .pDst = inv_fjac, .flag = 0U, .N = param_casted->num_cores, .nPE = pi_cl_cluster_nb_cores()};
        //pi_cl_team_fork(pi_cl_cluster_nb_cores(), plp_mat_inv_f32p_xpulpv2, (void *)&args);	
		pi_cl_team_fork(0, plp_mat_inv_f32p_xpulpv2, (void *)&args);
		//MatMul(fvec, inv_fjac, p, p->num_cores, p->num_cores, 1);
		plp_mat_mult_instance_f32 args2 = {
            .pSrcA = inv_fjac, .pSrcB = p, .M = param_casted->num_cores, .N = param_casted->num_cores, .O = 1, .nPE = pi_cl_cluster_nb_cores(), .pDstC = fvec
        };
        pi_cl_team_fork(0, plp_mat_mult_f32p_xpulpv2, (void *)&args2);
        errx=0.0; //Check root convergence.
        for (i=0;i<n;i++) //Update solution.
        {
            errx += fabs(fvec[i]);
            x[i] += fvec[i];
            printf("Xd[%d]: ", i);
            printFloat(x[i]);

           	varValue value1 = x[i] - lim_inf;
        	varValue value2 = x[i] - lim_sup;
        	printf(" - ");
        	printFloat(value1);
        	printf(" - ");
        	printFloat(value2);
    		printf("\n\r");

            if (value1 < 0)
            {
                diverged = 1;            
                if (value1 > divergence_factor)
                    divergence_factor = value1;

            } else if (value2>0) 
            {
                diverged = 1;
                varValue value2_app = value2;
                if(value2_app <0)
                	value2_app = -value2_app;
                if (value2_app > divergence_factor)
                    divergence_factor = value2;
            }
        }		
        #endif //GSL_PARALLEL

        if (diverged)
        {
            printf("\n\t\tRESET!\n\r\t");
            printFloat(divergence_factor);
            printf("\n\n\n\n\r");
            k = -1; //todo decide if I want it to consider whole
            /*
            for (i=0;i<n;i++)
            {
                //saved_init[i] -= (lim_sup-lim_inf)/divergence_factor;
                saved_init[i] -= (0.1)*divergence_factor;
                x[i] = saved_init[i];
                printf("newX[%d] = %f\n",i,x[i]);
            }
            */
            if (divergence_factor > 0)
            {
                for (i=0;i<n;i++)
                {
                    saved_init[i] -= 0.1;
                    x[i] = saved_init[i];
                    printf("newX[%d] = ",i);
                    printFloat(x[i]);
                    printf("\n\r");
                }
            } else //(divergence_factor < 0)
            {
                for (i=0;i<n;i++)
                {
                    saved_init[i] += 0.1;
                    x[i] = saved_init[i];
                    printf("newX[%d] = ",i);
                    printFloat(x[i]);
                    printf("\n\r");
                }
            }

        }
        if (errx <= tolx) break; //FREERETURN //TODO:
    }
    
    //FREERETURN
}

void bisec(int ntrial, varValue x[], int n, varValue tolx, varValue tolf, varValue* lim_inf, varValue* lim_sup, void *param,
    void (*usrfun)(varValue *,int ,varValue *,varValue(*)[PCF_CORES_MAX], void*))
/*
Given an initial guess x[1..n] for a root in n dimensions, take ntrial Newton-Raphson steps
to improve the root. Stop if the root converges in either summed absolute variable increments
tolx or summed absolute function values tolf.
*/
{
    int k,i;
    varValue errx,errf,d;

    #ifndef OPTIMIZATION_FUNC
    varValue fvec[PCF_CORES_MAX];
    varValue saved_fvec[PCF_CORES_MAX];

    varValue l_inf[PCF_CORES_MAX];
    varValue l_sup[PCF_CORES_MAX];
    #else
    varValue fvec[PCF_CORES_MAX+1];
    varValue saved_fvec[PCF_CORES_MAX+1];

    varValue l_inf[PCF_CORES_MAX+1];
    varValue l_sup[PCF_CORES_MAX+1];
    #endif

    //init
    //Check for feasbility pt.1
    for (i=0;i<n;i++)
    {
        x[i] = lim_sup[i];
    }
    usrfun(x,n,fvec,NULL, param);

    //lower init
    for (i=0;i<n;i++)
    {
        l_inf[i] = lim_inf[i];
        l_sup[i] = lim_sup[i];
        x[i] = lim_inf[i];
    }
    usrfun(x,n,saved_fvec,NULL, param);

    //Check for feasbility pt.2: boundaries
    for (i=0;i<n;i++)
    {
        int sign_n = (fvec[i] > 0) ? 1 : 0;
        int sign_o = (saved_fvec[i] > 0) ? 1 : 0;
        if (sign_n == sign_o)
        {
            //TODO Improve/optimize this:
            //Cuz I still need it to compute softmax, BUT
            //I should not compute f(x) and other stuff
            if (sign_n) // >0
            {
                //take the min:
                l_inf[i] = lim_inf[i];
                l_sup[i] = lim_inf[i];
                x[i] = lim_inf[i];

                //TODO: add pw difference in the bucket
            }
            else
            {
                //TODO Improve/optimize this:
                //Cuz I don't need to compute softmax, and
                //I should not compute f(x) and other stuff

                //Take the max:
                l_inf[i] = lim_sup[i];
                l_sup[i] = lim_sup[i];
                x[i] = lim_sup[i];

                //TODO: add pw difference in the bucket
            }
        }
    }

    #if (MEASURE_ACTIVE == 1)
	#ifdef PCF_USE_CLUSTER
	timerBuffer_c[Timerindex_c++] = (Timer_Data_t) {'1', get_timer_cnt_cl(timer0_id)};
	#else
	timerBuffer[Timerindex++] = (Timer_Data_t) {'1', lMeasureReadCsr( MEASURE_ZEROING)};
	#endif
	#endif

    for (k=0;k<ntrial;k++) {

        for (i=0;i<n;i++) {x[i] = (l_sup[i]+l_inf[i])/2;  /*printf("X[%d] = %f\n\r", i, x[i]);*/}
        //Right-hand side of linear equations.
        usrfun(x,n,fvec,NULL, param); //User function supplies function values at x in fvec and Jacobian matrix in fjac.

        //Check
        errf=0.0; //Check function convergence.
        errx=0.0; //Check root convergence.
        for (i=0;i<n;i++){ errf += fabs(fvec[i]); errx += fabs(l_sup[i]-x[i]);}
        if (errf <= tolf) break; //FREERETURN //TODO
        if (errx <= tolx) break; //FREERETURN //TODO

        //GO on
        for (i=0;i<n;i++)
        {
            int sign_n = (fvec[i] > 0) ? 1 : 0;
            int sign_o = (saved_fvec[i] > 0) ? 1 : 0;
            if ( sign_n == sign_o )
                {l_inf[i] = x[i]; saved_fvec[i] = fvec[i];} //TODO: improve this to remove saved_fvec[i]=, and instead alternate between the teo
                //l_sup[i] = x[i];
            else
                l_sup[i] = x[i];
                //{l_inf[i] = x[i]; saved_fvec[i] = fvec[i];}
            
        }

        #if (MEASURE_ACTIVE == 1)
        #ifdef PCF_USE_CLUSTER
        timerBuffer_c[Timerindex_c++] = (Timer_Data_t) {'2', get_timer_cnt_cl(timer0_id)};
        #else
        timerBuffer[Timerindex++] = (Timer_Data_t) {'2', lMeasureReadCsr( MEASURE_ZEROING)};
        #endif
        #endif
    }
    //printf("Steps: %d\n\r", k);
    
    //FREERETURN
}

/*
void solve_linear()
{
    ludcmp(a,n,indx,&d);
    lubksb(a,n,indx,b);
}
*/
/*
float
fastpow2 (float p)
{
  float offset = (p < 0) ? 1.0f : 0.0f;
  float clipp = (p < -126) ? -126.0f : p;
  int w = clipp;
  float z = clipp - w + offset;
  union { uint32_t i; float f; } v = { (uint32_t)( (1 << 23) * (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z) ) };

  return v.f;
}

float
fastexp (float p)
{
  return fastpow2 (1.442695040f * p);
}

varValue softmax(varValue *x, varValue alpha, varFor n)
{
	varValue res = VD_ZERO;
	varValue sumexp = VD_ZERO;
	for (varFor i = 0; i < n; i++)
	{
		res += x[i] * (varValue)fastexp(alpha*x[i]); //TODO: expf? o exp?
	}
	for (varFor i = 0; i < n; i++)
	{
		sumexp += (varValue)fastexp(alpha*x[i]); //TODO: expf? o exp?
	}

	return (res/sumexp);
}

varValue softmax_square(varValue *x, varValue alpha, varFor n)
{
	varValue res = softmax(x, alpha, n);

	return (res*res);
}

varValue der_softmax_square(varValue *x, varValue xj, varValue alpha, varFor n)
{
	varValue sumexp_xi = VD_ZERO;
	varValue sumexp = VD_ZERO;
	for (varFor i = 0; i < n; i++)
	{
		sumexp_xi += x[i] * (varValue)fastexp(alpha*x[i]); //TODO: expf? o exp?
	}
	for (varFor i = 0; i < n; i++)
	{
		sumexp += (varValue)fastexp(alpha*x[i]); //TODO: expf? o exp?
	}

	varValue softmax = sumexp_xi/sumexp;

	varValue res = 2 * softmax *
			(varValue)fastexp(alpha*xj)/sumexp *	//TODO: expf? o exp?
			(1 + alpha*(xj-softmax));

	return res;
} */


//TODO: license these 2 functions below!!
//https://github.com/akohlmey/fastermath
float
fastpow2 (float p)
{
  float offset = (p < 0) ? 1.0f : 0.0f;
  float clipp = (p < -126) ? -126.0f : p;
  int w = clipp;
  float z = clipp - w + offset;
  union { uint32_t i; float f; } v = { (uint32_t)( (1 << 23) * (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z) ) };

  return v.f;
}

float
fastexp (float p)
{
  return fastpow2 (1.442695040f * p);
}

varValue alpha_softmax = 10;

varValue softmax(varValue *x, varValue alpha, varFor n)
{
	varValue res = VD_ZERO;
	varValue sumexp = VD_ZERO;
	for (varFor i = 0; i < n; i++)
	{
		res += x[i] * (varValue)fastexp(alpha*x[i]); //TODO: expf? o exp?
	}
	for (varFor i = 0; i < n; i++)
	{
		sumexp += (varValue)fastexp(alpha*x[i]); //TODO: expf? o exp?
	}

	return (res/sumexp);
}

varValue softmax_square(varValue *x, varValue alpha, varFor n)
{
	varValue res = softmax(x, alpha, n);

	return (res*res);
}

varValue der_softmax_square(varValue *x, varValue xj, varValue alpha, varFor n)
{
	varValue sumexp_xi = VD_ZERO;
	varValue sumexp = VD_ZERO;
	for (varFor i = 0; i < n; i++)
	{
		sumexp_xi += x[i] * (varValue)fastexp(alpha*x[i]); //TODO: expf? o exp?
	}
	for (varFor i = 0; i < n; i++)
	{
		sumexp += (varValue)fastexp(alpha*x[i]); //TODO: expf? o exp?
	}

	varValue softmax = sumexp_xi/sumexp;

	varValue res = 2 * softmax *
			(varValue)fastexp(alpha*xj)/sumexp *	//TODO: expf? o exp?
			(1 + alpha*(xj-softmax));

	return res;
}



//design the fdf:
void pwrfunc(float *F, int n, float *f, float(*J)[PCF_CORES_MAX], void* p)
{
    /*const*/ //varValue F[PCF_CORES_MAX];
	//parameters:
	struct coupling_p2f *param = (struct coupling_p2f*)p;
	//const param_a = ....
	varFor num_cores = param->num_cores;

    varValue maxF = 0.5; //TODO Fmin
    for (varFor i=0; i < num_cores; i++)
    {
        if (F[i] > maxF)
        {
            maxF = F[i];
        }
    }

	for (varFor i = 0; i < num_cores; i++)
	{
        //#elif defined(USE_MONTE_CIMONE)
		f[i] = (param->ptr_wl[i] * F[i] /0.9f/0.9f * 
                    (maxF*param->alpha_vdd + param->aoffset_vdd)*(maxF*param->alpha_vdd + param->aoffset_vdd) ) + 
                    (param->k_vdd_stat*(maxF*param->alpha_vdd + param->aoffset_vdd)) + param->k_stat 
                     - param->ptr_Pc[i];
	}

	for (varFor i = num_cores; i < PCF_CORES_MAX; i++)
	{
		f[i] = 0; // x = 0;
	}

	return;
}
#ifndef OPTIMIZATION_FUNC
void usrfunc(float *F, int n, float *f, float(*J)[PCF_CORES_MAX], void* p)
{
	/*const*/ //varValue F[PCF_CORES_MAX];
	//parameters:
	struct coupling_p2f *param = (struct coupling_p2f*)p;
	//const param_a = ....
	float alpha_square_vdd = param->alpha_vdd*param->alpha_vdd; //pre-compute the square.
	varFor num_cores = param->num_cores;

	varValue der_softmax_square_mem[PCF_CORES_MAX];
	//varValue softmax_square_mem = softmax_square(F, alpha_softmax, num_cores);
    varValue softmax_mem;
    varValue softmax_square_mem;
    if(J!=NULL){
    softmax_mem = softmax(F, alpha_softmax, num_cores);
    softmax_square_mem = softmax_mem*softmax_mem;
    }
    else
    {
        varValue max_search = 0.5; //TODO Fmin
        for (varFor i=0; i < num_cores; i++)
        {
            if (F[i] > max_search)
            {
                max_search = F[i];
            }
        }
        softmax_mem = max_search;
        softmax_square_mem = max_search*max_search;
    }
	//printf("Smooth max: ");
	//printFloat(softmax_square_mem);
	//printf("\n\r");
	varValue ki[PCF_CORES_MAX];

	for (varFor i = 0; i < num_cores; i++)
	{
		if(J!=NULL){
		der_softmax_square_mem[i] = der_softmax_square(F, F[i], alpha_softmax, num_cores);
		}
		//#elif defined(USE_MONTE_CIMONE)
		ki[i] = (param->ptr_wl[i]*F[i]*alpha_square_vdd) / 0.9f / 0.9f;
	}

	for (varFor i = 0; i < num_cores; i++)
	{
		f[i] = ki[i]*softmax_square_mem + (param->k_vdd_stat*param->alpha_vdd*softmax_mem) + param->k_stat - param->ptr_Pc[i];
	}
	for (varFor i = num_cores; i < PCF_CORES_MAX; i++)
	{
		f[i] = 0; // x = 0;
	}

	if(J!=NULL){
    //TODO: FIX THIS!!!!!!! after change to V^2!
	for (varFor i = 0; i < num_cores; i++)
	{
		//printf("row %d\n\r", i);
		for (varFor j = 0; j < PCF_CORES_MAX; j++)
		{
			if (i==j)
			{
				//product of derivatives
				J[i][j] =  
					param->ptr_wl[i]*alpha_square_vdd*softmax_square_mem + 
				//	param->ptr_wl[i]*alpha_square_vdd * F[i] * der_softmax_square(F, F[j], alpha_softmax, num_cores) );
					ki[i]*der_softmax_square_mem[j];

				//printf("colum %d\n\r", j);
				//printFloat(J[i][j]);
				//printf("\n\r");
			}
			else if (j < param->num_cores)
			{
				J[i][j] =  
				//	param->ptr_wl[i]*alpha_square_vdd * F[i] * der_softmax_square(F, F[j], alpha_softmax, num_cores) );
					ki[i]*der_softmax_square_mem[j];

				//printf("colum %d\n\r", j);
				//printFloat(J[i][j]);
				//printf("\n\r");
			}
			else
			{
				J[i][j] = 0; //TBC: 0??
			}

		}
		//printf("\n\r");
	}
	//printf("\n\r");

    for (varFor i = num_cores; i < PCF_CORES_MAX; i++)
	{
		for (varFor j = 0; j < PCF_CORES_MAX; j++)
		{
			J[i][j] = 0; //TBC: 0??
		}
    }
	}

	return;
}
#else //not OPTIMIZATION_FUNC
void usrfunc(float *F, int n, float *f, float(*J)[PCF_CORES_MAX], void* p)
{
	/*const*/ //varValue F[PCF_CORES_MAX];
	//parameters:
	struct coupling_p2f *param = (struct coupling_p2f*)p;
	//const param_a = ....
	float alpha_square_vdd = param->alpha_vdd*param->alpha_vdd; //pre-compute the square.
	varFor num_cores = param->num_cores;

    varValue Fv = F[num_cores];
    varValue sum = VD_ZERO;
    varValue tolopt = 0.08 * num_cores; //TODO

	for (varFor i = 0; i < num_cores; i++)
	{
		//#elif defined(USE_MONTE_CIMONE)
        varValue diff = F[i]-Fv;
		f[i] = (param->ptr_wl[i]/0.9f/0.9f * (diff<0?F[i]:Fv) * alpha_square_vdd*Fv) + 
                    (param->k_vdd_stat*param->alpha_vdd*Fv) + param->k_stat 
                     - param->ptr_Pc[i];

        //optimization
        varValue alpha_importance = 1.0; //TODO
        //sum += alpha_importance * (diff>0?0:-diff); //TBC: zero
        //diff = param->ptr_target[i] - F[i];
        sum += alpha_importance*f[i];
	}

    //optimization
    f[num_cores] = sum>tolopt?sum-tolopt:sum;
    printf("opt: ");
    printFloat(sum - tolopt);
    printf("\n\r");
    printf("Fv: ");
    printFloat(Fv);
    printf("\n\r");


	for (varFor i = num_cores+1; i < PCF_CORES_MAX; i++)
	{
		f[i] = 0; // x = 0;
	}

	return;
}
#endif //not OPTIMIZATION_FUNC

//PARALLEL
/**
  @ingroup MatInv
 */

/**
  @addtogroup MatInvKernels
  @{
 */

/**
   @brief Parallel matrix inversion of 32-bit floating-point matrices kernel for XPULPV2 extension.
   @param[in]  args  pointer to plp_mat_inv_instance_f32 struct initialized by
                    plp_mat_inv_f32_parallel
   @return     0: Success, 1: Matrix is singular
   @warn Not yet implemented
*/

int plp_mat_inv_f32p_xpulpv2(void *args) {

    int core_id = (int)pi_core_id();

    plp_mat_inv_instance_f32 *a = (plp_mat_inv_instance_f32 *)args;
    float *__restrict__ pSrc = a -> pSrc;
    float *__restrict__ pDst = a -> pDst;
    uint32_t *flag = a -> flag;
    int N = a -> N;
    int nPE = a -> nPE;
    float *pSrcT1, *pSrcT2;                         /* Temporary input data matrix pointer */
    float *pDstT1, *pDstT2;                         /* Temporary output data matrix pointer */
    float *pPivotRowIn;                             /* Temporary input and output data matrix pointer */
    float *pPRT_in, *pPivotRowDst, *pPRT_pDst;      /* Temporary input and output data matrix pointer */
    float Xchg, in = 0.0f, in1;                     /* Temporary input values  */
    int i, rowCnt, j, loopCnt, k, l;   /* loop counters */
    int M = N;                         /* M is the number of rows. However, the matirces must be square. */
    pDstT1 = pDst;                          /* Working pointer for destination matrix */

    /* CREATE IDENTITY MATRIX */

    for (i = core_id; i < M; i += nPE) {
        for (j = 0; j < M; j++) {
            pDstT1[i * M + j] = (float)(i == j);
        }
    }
    
    pi_cl_team_barrier();

    /* Loop over the number of columns of the input matrix.
       All the elements in each column are processed by the row operations */
    loopCnt = N;
    l = 0U; // Column index
    while (loopCnt > 0U) {

        /* SEARCH FOR 0 AND SWITCH ROWS */

        pSrcT1 = pSrc + (l * N); // Input pivot
        pDstT1 = pDst + (l * N); // Destination pivot

        in = *pSrcT1;
        k = 1U;
        /* Check if the pivot element is zero */
        if (*pSrcT1 == 0.0f) {
            /* Loop over the number rows present below */
            for (i = (l + 1U) + core_id; i < M; i += nPE) {
                pSrcT2 = pSrcT1 + (N * i);
                /* Check if there is element to exchange */
                if (*pSrcT2 != 0.0f)
                    *flag = k;
                if (*flag != 0) {
                    pSrcT2 = pSrcT1 + (N * *flag + l);
                    pDstT2 = pDstT1 + (N * *flag);
                    /* Loop over number of columns
                     * to the right of the pilot element */
                    for (j = core_id; j < N - l; j += nPE) {
                        /* Exchange the row elements of the input matrix */
                        Xchg = pSrcT2[j];
                        pSrcT2[j] = pSrcT1[j];
                        pSrcT1[j] = Xchg;
                    }
                    pSrcT1 += N - l;
                    pSrcT2 += N - l;
                    /* Loop over number of columns of the destination matrix */
                    for(j = core_id; j < N; j += nPE) {
                        /* Exchange the row elements of the destination matrix */
                        Xchg = pDstT2[j];
                        pDstT2[j] = pDstT1[j];
                        pDstT1[j] = Xchg;
                    }
                    pDstT2 += N;
                    pDstT1 += N;
                    break;
                }
            }
            k++;
            pi_cl_team_barrier();
        }

        /* Update the status if the matrix is singular */
        if ((*flag == 0U) && (in == 0.0f)) {
            return 1;
        }

        /* DIVIDE BY PIVOT */

        /* Points to the pivot row of input and destination matrices */
        pPivotRowIn = pSrc + (l * N);
        pPivotRowDst = pDst + (l * N);
        /* Temporary pointers to the pivot row pointers */
        pSrcT1 = pPivotRowIn;
        pSrcT2 = pPivotRowDst;
        /* Pivot element of the row */
        in = *pPivotRowIn;
        /* Loop over number of columns to the right of the pilot element */
        for(j = core_id; j < N - l; j += nPE) {
            in1 = pSrcT1[j];
            pSrcT1[j] = in1 / in;
        }
        /* Loop over number of columns of the destination matrix */
        for(j = core_id; j < N; j += nPE) {
            in1 = pSrcT2[j];
            pSrcT2[j] = in1 / in;
        }
        pi_cl_team_barrier();

        /*REPLACE ROWS */

        pSrcT1 = pSrc + core_id * N;
        pSrcT2 = pDst + core_id * N;
        i = core_id;
        k = M;
        for(k = core_id; k < M; k += nPE) {
            if (i != l) {
                /* Element of the reference row */
                in = *pSrcT1;
                /* Working pointers for input and destination pivot rows */
                pPRT_in = pPivotRowIn;
                pPRT_pDst = pPivotRowDst;
                /* Loop over the number of columns to the right of the pivot element,
                   to replace the elements in the input matrix */
                for (j = 0; j < N - l; j++) {
                    in1 = pSrcT1[j];
                    pSrcT1[j] = in1 - (in * pPRT_in[j]);
                }
                /* Loop over the number of columns to
                   replace the elements in the destination matrix */
                for (j = 0; j < N; j++) {
                    in1 = pSrcT2[j];
                    pSrcT2[j] = in1 - (in * pPRT_pDst[j]);
                }
            }
            i += nPE;
            pSrcT1 += nPE * N;
            pSrcT2 += nPE * N;
        }
        /* Increment the input pointer */
        pSrc++;
        /* Decrement the loop counter */
        loopCnt--;
        /* Increment the index modifier */
        l++;
        pi_cl_team_barrier();
    }
    
    if ((*flag == 0) && (in == 0.0f)) {
        for (i = 0; i < M * N; i++) {
            if (pSrc[i] != 0.0f)
                break;
        }
        if (i == M * N)
            return 1;
        pi_cl_team_barrier();
    }

    //hal_eu_mutex_lock(0);
	//printf("(%ld, %ld) Done inversion\r\n", pi_cluster_id(),
    //          pi_core_id());
 	//hal_eu_mutex_unlock(0);
    return 0;
}

void plp_mat_mult_f32p_xpulpv2(void *args) {

    int core_id = pi_core_id();

    plp_mat_mult_instance_f32 *a = (plp_mat_mult_instance_f32 *)args;

    const float *__restrict__ pSrcA = a->pSrcA;
    const float *__restrict__ pSrcB = a->pSrcB;
    uint32_t M = a->M;
    uint32_t N = a->N;
    uint32_t O = a->O;
    uint32_t nPE = a->nPE;
    float *__restrict__ pDstC = a->pDstC;

#define BASIC_VERSION // if used don't forget to also use the undefine at end of file
#ifdef BASIC_VERSION

    uint32_t m, n, o;

    for (m = core_id; m < M; m += nPE) {
        for (o = 0; o < O; o++) {
            float sum = 0;
            for (n = 0; n < N; n++) {
                sum = sum + pSrcA[m * N + n] * pSrcB[n * O + o];
            }
            pDstC[m * O + o] = sum;
        }
    }
#else
    // TODO: Hackathon
#endif
#undef BASIC_VERSION

    //hal_eu_mutex_lock(0);
	//printf("(%ld, %ld) Done matmul\r\n", pi_cluster_id(),
    //          pi_core_id());
 	//hal_eu_mutex_unlock(0);

    pi_cl_team_barrier();
}

//#endif //USE_NEWTON_RAPSON

