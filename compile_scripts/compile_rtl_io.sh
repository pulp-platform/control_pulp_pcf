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

source $FREERTOS_PROJ_ROOT/env/control-pulp.sh

export PCF_TARGET=PMS_CV32
export PCF_PLATFORM=DATE_PAPER

cd $PCF_PATH

if [[ $1 = "blocking" ]]
then
    make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" \
    			MAKE_FLAGS="-UUSE_FIXED_VOLTAGE -UUSE_MONTE_CIMONE \
    			-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DPRINTF_ACTIVE -DPCF_USE_CLUSTER -UPCF_USE_CLUSTER_PARALL \
    			-DTASKS_PERIOD_MULTIPLIER=200 -DPCF_FREERTOS_TICK_RATE_HZ=5 -DDEFAULT_SYSTEM_CLOCK=20000000u \
    			-DCI_TEST -DCI_TEST_ITERATION=10 -UMEASURE_ACTIVE \
    			" all 

elif [[ $1 = "async" ]]
then
    make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" \
    			MAKE_FLAGS="-UUSE_FIXED_VOLTAGE -UUSE_MONTE_CIMONE \
    			-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DPRINTF_ACTIVE -DPCF_USE_CLUSTER -DPCF_USE_CLUSTER_PARALL \
    			-DTASKS_PERIOD_MULTIPLIER=200 -DPCF_FREERTOS_TICK_RATE_HZ=5 -DDEFAULT_SYSTEM_CLOCK=20000000u \
    			-DCI_TEST -DCI_TEST_ITERATION=10 -UMEASURE_ACTIVE \
    			" all 
else

	#nothing, NO PRINTF
	#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DTASKS_PERIOD_MULTIPLIER=10 -UCI_TEST -DCI_TEST_ITERATION=1000 -UMEASURE_ACTIVE -UPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=125 -DDEFAULT_SYSTEM_CLOCK=20000000u" all

	#nothing, with printf
	#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DTASKS_PERIOD_MULTIPLIER=200 -DCI_TEST -DCI_TEST_ITERATION=100 -UMEASURE_ACTIVE -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=5 -DDEFAULT_SYSTEM_CLOCK=20000000u" all 
	#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" \
	#			MAKE_FLAGS="-UUSE_FIXED_VOLTAGE -DUSE_MONTE_CIMONE \
	#			-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DPRINTF_ACTIVE -UPCF_USE_CLUSTER -UPCF_USE_CLUSTER_PARALL \
	#			-DTASKS_PERIOD_MULTIPLIER=200 -DPCF_FREERTOS_TICK_RATE_HZ=5 -DDEFAULT_SYSTEM_CLOCK=20000000u \
	#			-DCI_TEST -DCI_TEST_ITERATION=10 -UMEASURE_ACTIVE \
	#			" all 

	questa-2022.3 make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" \
				MAKE_FLAGS="-UUSE_FIXED_VOLTAGE -DUSE_MONTE_CIMONE \
				-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DPRINTF_ACTIVE -UPCF_USE_CLUSTER -UPCF_USE_CLUSTER_PARALL \
				-DTASKS_PERIOD_MULTIPLIER=200 -DPCF_FREERTOS_TICK_RATE_HZ=5 -DDEFAULT_SYSTEM_CLOCK=20000000u \
				-DCI_TEST -DCI_TEST_ITERATION=10 -UMEASURE_ACTIVE \
				" all run gui=1


	#CI TEST, with PRINTF
	#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DTASKS_PERIOD_MULTIPLIER=5 -DCI_TEST -DCI_TEST_ITERATION=1000 -UMEASURE_ACTIVE -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=250 -DDEFAULT_SYSTEM_CLOCK=20000000u" all

	#Cycles print
	#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DTASKS_PERIOD_MULTIPLIER=10 -UCI_TEST -DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=20 -DMEASURE_N_OUTPUTS=64 -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=125 -DDEFAULT_SYSTEM_CLOCK=20000000u" all
fi


#cd

# Flags added
# PCF_ASYNC
# ADRIEL_BUG_FIXES (I'm unsure / have questions about my bugfix)
# USE_INSTRUCTIONS_COMPOSITION (caused compilation errors without)
