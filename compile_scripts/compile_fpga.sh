#!/bin/bash

##########################################################################
#
# Copyright 2023 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
# Author: Giovanni Bambini (gv.bambini@gmail.com)
#
##########################################################################

if [ -z "${FREERTOS_PROJ_ROOT+x}" ]; then
    echo "FREERTOS_PROJ_ROOT not setup, assumed to be in the control-pulp repository"
	export FREERTOS_PROJ_ROOT="../freertos"
else
    echo $FREERTOS_PROJ_ROOT
fi

source $FREERTOS_PROJ_ROOT/env/control-pulp-fpga.sh

export PCF_TARGET=PMS_CV32
export PCF_PLATFORM=FPGA_ZCU102

if [ -z "${PCF_RISCV_TC_PATH+x}" ]; then
    echo "Please setup PCF_RISCV_TC_PATH to the path of the RISCV Toolchain"
else
    echo $PCF_RISCV_TC_PATH
fi

if [ -z "${PCF_PATH+x}" ]; then
	echo "PCF_PATH not defined, assumed to be already in the folder"
else
	cd $PCF_PATH
fi

if [[ $1 = "blocking" ]]
then
	make clean RISCV=$PCF_RISCV_TC_PATHCFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" \
    			MAKE_FLAGS="-UUSE_FIXED_VOLTAGE -DUSE_MONTE_CIMONE \
    			-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DPRINTF_ACTIVE -DPCF_USE_CLUSTER -UPCF_USE_CLUSTER_PARALL \
    			-DTASKS_PERIOD_MULTIPLIER=200 -DPCF_FREERTOS_TICK_RATE_HZ=5 -DDEFAULT_SYSTEM_CLOCK=20000000u \
    			-UCI_TEST -DCI_TEST_ITERATION=100 -UMEASURE_ACTIVE \
				-DTASKS_PERIOD_MULTIPLIER_P2=1 -UCTRL_DEBUG \
    			" all 
	echo $1

elif [[ $1 = "async" ]]
then
	make clean RISCV=$PCF_RISCV_TC_PATH CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" \
    			MAKE_FLAGS="-UUSE_FIXED_VOLTAGE -DUSE_MONTE_CIMONE \
    			-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DPRINTF_ACTIVE -DPCF_USE_CLUSTER -DPCF_USE_CLUSTER_PARALL \
    			-DTASKS_PERIOD_MULTIPLIER=25 -DPCF_FREERTOS_TICK_RATE_HZ=40 -DDEFAULT_SYSTEM_CLOCK=20000000u \
    			-UCI_TEST -DCI_TEST_ITERATION=100 -UMEASURE_ACTIVE \
				-DTASKS_PERIOD_MULTIPLIER_P2=1 -UCTRL_DEBUG \
    			" all 

	echo $1
else
	
	#make clean RISCV="/opt/riscv" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" \
	#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" \
	make clean RISCV=$PCF_RISCV_TC_PATH CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" \
				MAKE_FLAGS="-UUSE_FIXED_VOLTAGE -DUSE_MONTE_CIMONE \
				-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DPRINTF_ACTIVE -UPCF_USE_CLUSTER -UPCF_USE_CLUSTER_PARALL \
				-DTASKS_PERIOD_MULTIPLIER=25 -DPCF_FREERTOS_TICK_RATE_HZ=40 -DDEFAULT_SYSTEM_CLOCK=20000000u \
				-UCI_TEST -DCI_TEST_ITERATION=100 -UMEASURE_ACTIVE \
				-DTASKS_PERIOD_MULTIPLIER_P2=1 -UCTRL_DEBUG \
				" all 

	echo "no cluster"

	#CI TEST, with PRINTF
	#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DTASKS_PERIOD_MULTIPLIER=5 -DCI_TEST -DCI_TEST_ITERATION=1000 -UMEASURE_ACTIVE -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=250 -DDEFAULT_SYSTEM_CLOCK=20000000u" all

	#Cycles print
	#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DTASKS_PERIOD_MULTIPLIER=10 -UCI_TEST -DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=20 -DMEASURE_N_OUTPUTS=64 -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=125 -DDEFAULT_SYSTEM_CLOCK=20000000u" all
fi


#cd


