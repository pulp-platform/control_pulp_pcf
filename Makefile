
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


# Common code
#------------------------------------

# ARE WE IN DEVELOPMENT ?
ACTIVE_DEVELOP = 1
###

APP = test
APP_SRCS 	= ./PCF_Core/pcf_main.c
APP_SRCS 	+= ./PCF_Core/pcf_taskFunctions.c ./PCF_Core/pcf_tasks.c
APP_SRCS 	+= ./PCF_Core/Secure/sec_tasks.c ./PCF_Core/Secure/sec_functions.c ./PCF_Core/Secure/sec_globalProtectedVar.c
APP_SRCS 	+= ./PCF_Core/Control/ctrl_functions.c ./PCF_Core/Control/ctrl_math.c ./PCF_Core/Control/ctrl_tasks.c

#APP_SRCS	+= ./freertosFromISR/freertos_taskFromISR.c

APP_CFLAGS  +=

# Include Paths
INC_PATH	+= ./PCF_Core ./PCF_Core/Config ./PCF_Core/Control ./PCF_Core/Secure
INC_PATH	+= ./PCF_Core/Target/Include ./PCF_Core/Target/Include/Debug
INC_PATH 	+= ./System_Implementation/Include/

######################################################
#TODO REMOVE
INC_PATH 	+= ./PCF_Core/Target/PMS_CV32

#INC_PATH	+= ./freertosFromISR

## Implementation
PCFP = $(PCF_PLATFORM)
ifeq ($(PCFP),)
PCFP = NO_IO
endif

APP_SRCS 	+= ./System_Implementation/$(PCFP)/imp_comms.c
INC_PATH	+= ./System_Implementation/$(PCFP)/

#Special Inclusions
#1
ifeq ($(PCFP),NO_IO)
APP_SRCS 	+= ./System_Implementation/$(PCFP)/imp_dataLib.c
endif

##Target
PCFT = $(PCF_TARGET)
ifeq ($(PCFT),)
PCFT = PMS_CV32
endif

APP_SRCS 	+= ./PCF_Core/Target/$(PCFT)/tgt_init.c
APP_SRCS 	+= ./PCF_Core/Target/$(PCFT)/tgt_port.c
APP_SRCS 	+= ./PCF_Core/Target/$(PCFT)/tgt_timer.c
APP_SRCS	+= ./PCF_Core/Target/$(PCFT)/tgt_cluster.c
ifeq ($(ACTIVE_DEVELOP),1)
APP_SRCS 	+= ./PCF_Core/Target/$(PCFT)/Debug/tgt_dbg_measure.c
APP_SRCS 	+= ./PCF_Core/Target/$(PCFT)/Debug/tgt_debug.c
APP_SRCS 	+= ./PCF_Core/Target/$(PCFT)/Debug/tgt_dbg_ci.c
APP_SRCS 	+= ./PCF_Core/Target/$(PCFT)/Debug/print_float.c
endif

#INC_PATH += /home/gbambini/control-pulp/fw/gsl_compiled/include
#CPPFLAGS += -L/home/gbambini/control-pulp/fw/gsl_compiled/lib 
#CPPFLAGS += /home/gbambini/control-pulp/fw/gsl_compiled/lib/libgsl.a /home/gbambini/control-pulp/fw/gsl_compiled/lib/libgslcblas.a 
#CPPFLAGS += -lgsl -lgslcblas -lm

INCLUDES = $(foreach f, $(INC_PATH), -I$f)

#APP_SRCS += $(shell find ./gsl-2.7.1 -name '*.c')
#INCLUDES += $(shell find ./gsl-2.7.1 -name '*.h')

# indicate this repository's root folder
# set some project specific path variables
ifndef FREERTOS_PROJ_ROOT
$(error "FREERTOS_PROJ_ROOT is unset. Run source env/platform-you-want.sh from \
	the freertos project's root folder.")
endif

# good defaults for many environment variables
include $(FREERTOS_PROJ_ROOT)/default_flags.mk

# rtos and pulp sources
include $(FREERTOS_PROJ_ROOT)/default_srcs.mk

# FreeRTOS Definitions:
CPPFLAGS += -DPCF_FREERTOS_TICK_RATE_HZ=1000
CPPFLAGS += -DDEFAULT_SYSTEM_CLOCK=500000000u

# ARE WE IN DEVELOPMENT ?
CPPFLAGS += -DDEBUG_ACTIVE -DPIDV2 -UTEST_GSL_LIBRARY -DUSE_NEWTON_RAPSON -UGSL_PARALLEL -DUSE_BISECT 
CPPFLAGS += -UUSE_CTRL_STRCT_INVERSE -UUSE_CTRL_STRCT_VB -UOPTIMIZATION_FUNC
#####

#Uncomment this to print information via printf
CPPFLAGS += -DPRINTF_ACTIVE
CPPFLAGS += -DDEBUG_PRINTF
CPPFLAGS += -DHRO_PRINTF
CPPFLAGS += -DALPHA_DEBUG

######

CPPFLAGS += -DTASKS_PERIOD_MULTIPLIER=1

# Uncomment to add debug option
#CPPFLAGS += -g

#Code Congiguration through Flags:
#CPPFLAGS += -DDATE_PAPER_FIXED_FETCH
#CPPFLAGS += -DUSE_INSTRUCTIONS_COMPOSITION

#To use float
CPPFLAGS += -DCONTROL_DATA_TYPE=1

#Uncomment this for the CI_TEST
CPPFLAGS += -DCI_TEST
#and the number of iterations
CPPFLAGS += -DCI_TEST_ITERATION=1


#Measurement test
CPPFLAGS += -UMEASURE_ACTIVE
#and the number of iterations for the measurement test
CPPFLAGS += -DMEASURE_N_ITERATION=20
#and the number of measurements per iteration for the measurement test
CPPFLAGS += -DMEASURE_N_OUTPUTS=64
#the type of measure
#TODO: sadly the MEASURE_CSR value is target dependant. Still I don't know how to do otherwise
CPPFLAGS += -DMEASURE_CSR=0 -DMEASURE_ZEROING=0

#####################
#Last One
#Make flags:
CPPFLAGS += $(MAKE_FLAGS)

# application name
PROG = Firmware

# application/user specific code
USER_SRCS = $(APP_SRCS)
CPPFLAGS += $(INCLUDES)

# user headers
CPPFLAGS += $(addprefix -I$(USER_DIR)/, ".")

# point to irq handler
CPPFLAGS += -DportasmHANDLE_INTERRUPT=vSystemIrqHandler
# For the cluster
CPPFLAGS += -DUSE_STDIO

#todo remove
#to fix compiler
RISCV = /usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1
CFLAGS = -Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32 -mno-pulp-hwloop
ASFLAGS = -Os -g3 -march=rv32imc_zfinx_xcorev -mabi=ilp32 -mno-pulp-hwloop

LDFLAGS += #-L/home/gbambini/control-pulp/fw/gsl_compiled/lib #-static -lgsl -lgslcblas #-lm
LDLIBS += #/home/gbambini/control-pulp/fw/gsl_compiled/lib/libgsl.a /home/gbambini/control-pulp/fw/gsl_compiled/lib/libgslcblas.a 
LDLIBS += #-lgsl -lgslcblas -lm -lm

# compile, simulation and analysis targets
include $(FREERTOS_PROJ_ROOT)/default_targets.mk
