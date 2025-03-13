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
export PCF_PLATFORM=NO_IO

cd $PCF_PATH

#CI TEST
#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DUSE_INSTRUCTIONS_COMPOSITION -DTASKS_PERIOD_MULTIPLIER=1 -DCI_TEST -DCI_TEST_ITERATION=10 -UMEASURE_ACTIVE -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=1000 -DDEFAULT_SYSTEM_CLOCK=100000000u" all run gui=1

make RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DTASKS_PERIOD_MULTIPLIER=5 -UCI_TEST -DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=10 -DMEASURE_N_OUTPUTS=72 -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=1000 -DDEFAULT_SYSTEM_CLOCK=100000000u -DPCF_USE_CLUSTER" clean all run gui=1

#Cycles print (Timer)
#make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DUSE_INSTRUCTIONS_COMPOSITION -DTASKS_PERIOD_MULTIPLIER=1 -UCI_TEST -DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=10 -DMEASURE_N_OUTPUTS=64 -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=1000 -DDEFAULT_SYSTEM_CLOCK=100000000u" all run

cd
