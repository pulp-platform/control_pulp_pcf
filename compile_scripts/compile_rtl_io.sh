
#!/bin/bash

source $FREERTOS_PROJ_ROOT/env/control-pulp.sh

export PCF_TARGET=PMS_CV32
export PCF_PLATFORM=DATE_PAPER

cd $PCF_PATH

if [[ $1 = "blocking" ]]
then
    make RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DTASKS_PERIOD_MULTIPLIER=5 -UCI_TEST -DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=10 -DMEASURE_N_OUTPUTS=72 -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=1000 -DDEFAULT_SYSTEM_CLOCK=100000000u -DPCF_USE_CLUSTER -UPCF_USE_CLUSTER_PARALL" clean all run
elif [[ $1 = "async" ]]
then
    make RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -DFEATURE_CLUSTER=1 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DTASKS_PERIOD_MULTIPLIER=5 -UCI_TEST -DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=10 -DMEASURE_N_OUTPUTS=72 -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=1000 -DDEFAULT_SYSTEM_CLOCK=100000000u -DPCF_USE_CLUSTER -DPCF_USE_CLUSTER_PARALL" clean all run
else
    #CI TEST
    #make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DTASKS_PERIOD_MULTIPLIER=1 -DCI_TEST -DCI_TEST_ITERATION=10 -UMEASURE_ACTIVE -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=1000 -DDEFAULT_SYSTEM_CLOCK=100000000u" all run

    #Cycles print (Timer)
    #here the interaction between the choice of PCF_FREERTOS_TICK_RATE_HZ and TASKS_PERIOD_MULTIPLIER reflect on the value of MEASURE_N_OUTPUTS.
    #So be careful to play with values.
    #make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32"                       ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DTASKS_PERIOD_MULTIPLIER=1 -UCI_TEST -DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=10 -DMEASURE_N_OUTPUTS=64 -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=1000 -DDEFAULT_SYSTEM_CLOCK=100000000u" all run

    #make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" ASFLAGS="-Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DMEMORY_FIXED_FETCH -DUSE_INSTRUCTIONS_COMPOSITION -DTASKS_PERIOD_MULTIPLIER=10 -UCI_TEST -DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=40 -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=125 -DDEFAULT_SYSTEM_CLOCK=20000000u" all

    ## --- new

    make clean RISCV="/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1" CFLAGS="-Og -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32 -D__PULP__=1 -DDEBUG" ASFLAGS="-Os -g3 -march=rv32imac_zfinx_xcorev -mabi=ilp32" MAKE_FLAGS="-DTASKS_PERIOD_MULTIPLIER=5 -UCI_TEST -DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=16 -DMEASURE_N_OUTPUTS=72 -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=1000 -DDEFAULT_SYSTEM_CLOCK=100000000u -UPCF_USE_CLUSTER -UPCF_USE_CLUSTER_PARALL" all run
    #echo "Please indicate 'blocking' or async'"
fi

cd

# Flags added
# PCF_ASYNC
# ADRIEL_BUG_FIXES (I'm unsure / have questions about my bugfix)
# USE_INSTRUCTIONS_COMPOSITION (caused compilation errors without)
