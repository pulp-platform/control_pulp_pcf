# PCF PROJECT
## AN OPEN-SOURCE POWER AND TEMPERATURE CONTROL FIRMWARE

This project is an open-source Power and Temperature firmware for Multi-Core CPUs, which aim to be scalable and exportable to many chips and platforms.

The full documentation can be found here: *currently in development*.

## Directory Structure
```
|-- PCF_Core            Contains all the code of the Firmware
    |-- Config          Contains the configuration headers and the #define to modify the code
    |-- Control         Contains all the Control part and the Math libraries
    |-- Secure          This is a "secure environment" with functions and tasks that have
                            high impact on security
    |-- Target          Contains the functions that are Target-dependent.
        |-- DEBUG       Contains files to help the debug and the auto-tests
|-- System_Impl         Contains the functions that are System/Platform-dependent
FreeRTOSConfig.h        File needed for FreeRTOS
MakeFile                The Makefile
|-- Others              A folder containing other files
|-- compile_scripts     A folder containing some bash script to compile the firmware with certain structures
```


## Setup Guide
You need to start from ControlPULP. Sadly the way the PCF code is compiled with the ControlPULP drivers, the ControlPULP FreeRTOS porting, and the ControlPULP necessary file for the RISC-V toolchain, make it tangled to the control-pulp repository. Thus you have to start from there.

First the ControlPULP Pre-requisite:
### GCC Toolchain:
Pre-requisite: (https://github.com/pulp-platform/riscv-gnu-toolchain)
```
sudo apt-get install autoconf automake autotools-dev curl python3 libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev expect
```
Cloning and compiling the toolchain never worked, thus, to get the toolchain you need to:
```
scp -r account_name@lagrev5.ee.ethz.ch:/usr/pack/riscv-1.0-kgf/pulp-gcc-2.6.0/ .
```

then do this each time you open the terminal, or add it to your bashrc file
```
export PATH=path_to_the_copied_folder/bin:$PATH
```

The other method that DOES NOT WORK IS:
	> Get the Repo: (tens of minutes)
	>```
	git clone --recursive https://github.com/pulp-platform/riscv-gnu-toolchain.git
	```
	>Compile the toolchain (2h+)
	>```
	./configure --prefix=/opt/riscv --with-arch=rv32imfcxpulpv3 --with-abi=ilp32 --enable-multilib
	make
	```
	>
	>```
	export PATH=/opt/riscv/bin:$PATH
	```
	>
	>Test it
	>```
	./configure --prefix=$HOME/riscv-toolchain-test --with-arch=rv32ima
	make report-gcc-newlib
	```

### ControlPULP

```
git clone git@iis-git.ee.ethz.ch:pms/control-pulp.git
```

Clone FreeRTOS
```
make freertos
```
In the ControlPULP repository *language*, FreeRTOS means:
 + FreeRTOS open source kernel
 + ControlPULP drivers
 + ControlPULP porting layer
 + Additional files for compilation
 
Now clone the PCF firmware. Changes in code and the Repo is going to be managed from here.
```
make fw
```
since ControlPULP Repo is not managed for some years, if you get some errors don't worry.
We also need to fix and update the repo. 
```
cd fw
git fetch origin
git checkout master
git pull origin master
```

Other things to do to initialize the control-pulp repo:
```
pip install --user pyelftools
./generate-scripts
```

## Compiling the Firmware

The PCF is setup to be compiled targetting different platform and devices. In particular:
* The variable **PCF_PLATFORM** has to be defined as the name of the folder of the System implementation: e.g. `export PCF_PLATFORM=FPGA_ZCU102` . Default value is NO_IO.
* The variable **PCF_TARGET** has to be defined as the name of the folder of the Target implementation: e.g. `export PCF_TARGET=PMS_RI5CY` . Default value is PMS_CV32.
Defaults can be changed in the Makefile
```
export PCF_TARGET=<name_of_the_folder_under_./PCF_Core/Target/)>
export PCF_PLATFORM=<name_of_the_folder_under_./System_Implementation)>
```

### For General Implementation:

After setting  **PCF_PLATFORM**, **PCF_TARGET**, other variables needs to be setup for a successful compilation:
```
export PCF_RISCV_TC_PATH=<your_riscv_toolchain_path>
export FREERTOS_PROJ_ROOT=<your_freertos_path>
export PCF_PATH=<your_PCF_path>
```
*Alternative, one could setup the **RISCV** variable directly instead of **PCF_RISCV_TC_PATH**.*

Then we need to:
```
source $FREERTOS_PROJ_ROOT/env/control-pulp.sh
make MAKE_FLAGS="-U<the_config_you_want_to_change> -D<the_config_you_want_to_change>" all
```

### Compilation Flags
There are several compilation flags that help configure the code in parametric way. These can be defined in 3 ways:
- In the Makefile file
- Passed in the **MAKE_FLAGS** variable when calling `$ make`
- ~~defined in the code~~

The definitions will have this type of priority:
**MAKE_FLAGS** \> Makefile \> Code

These parameters are active if defined (independently of the value), and inactive if undefined.
- Control:
	- `PIDV1`, `PIDV2`, `PIDV3`: define different parameters for the pid control. Currently the **PIDV2** is used. If none is defined, the default values are used. If more then one is defined, the chosen parameters are: `PIDV1` \> `PIDV2` \> `PIDV3`
	- `USE_CTRL_STRCT_INVERSE`: if defined, the control cascade is reversed with the thermal capping step first. If undefined the opposite default structure is used
	- `USE_CTRL_STRCT_VB`: if defined, the control is changed into parallel voting box. If undefined the default cascade structure is used.
	- `USE_CTRL_FUZZY`: if defined, the control algorithm is changed to Fuzzy. **Note**: to use the Fuzzy+iterative algorithm you also need to set `variable_voltage_solution` in `PCF_Core/Secure/sec_globalProtectedVar.c` at `VV_COUPLING_SOLUTION`
	**IMPORTANT TO NOTE!** do NOT define more than one of `USE_CTRL_STRCT_INVERSE`, `USE_CTRL_STRCT_VB`, and `USE_CTRL_FUZZY` at the same time, as the results is undefined.
	- `USE_BISECT`: if defined use the Newton-Rapson method for finding the root in the iterative Power-to-Frequency conversion step of the control. Preferred method.
	- `USE_NEWTON_RAPSON`: if defined use the Newton-Rapson method for finding the root in the iterative Power-to-Frequency conversion step of the control. To work, `USE_BISECT` has to be undefined, as the latter is the preferred method. Newton-Rapson method may be slow
	**IMPORTANT TO NOTE!** either bisec or Newton-Rapson method won't "activate" unless the variable `variable_voltage_solution` in `PCF_Core/Secure/sec_globalProtectedVar.c` is set to `VV_COUPLING_SOLUTION`.
	- ~~`USE_FIXED_VOLTAGE`: use a fixed voltage for the control~~ no longer used in the code
	- `USE_RHEA`: use RHEA power model parameters! 
	- `USE_MONTE_CIMONE`: use Monte Cimone power model parameters. 
	`USE_RHEA` \> `USE_MONTE_CIMONE`, is none defined, the old "Hipeac" power model parameters are used. **IMPORTANT!** To have good control behavior, these has to be the same/similar to the parameters of the power model of the simulation
- Simulation:
	- `MEMORY_FIXED_FETCH`: legacy parameter. Should be kept always **DEFINED**. If not defined, the PCF will start to print all values one after the other instead at fixed memory addresses. This was done to debug PCF behavior and PCF comunication. It will break the simulation, if used with an active simulation.
	- `USE_INSTRUCTIONS_COMPOSITION`: if defined, it is expected that the simulation passes `int` percentages of the different workload levels. When undefined, it is expected to pass directly the Ceff parameter (`float`). **IMPORTANT!** to note that this second behavior may be currently broken in the code and needs to be restored in the `fMultiInputProcess()` function in `ctrl_functions.c`, and tracked along the code
	- `ALPHA_DEBUG`: if defined, the PCF will also send the alpha information regarding the power reduction control step, to the simulation, to be recorded for debug. If not needed for debugging of verification purposes, it is advised to disable this parameter (here and in the simulation) to reduce the overheads and the amount of information
- Firmware:
	- `PRINTF_ACTIVE`: if undefined, automatically disabled most of the printf in the code. This is done since printf are very time-costly, and thus, if they are enabled, they change the behavior of the periodic execution of the real-time code. Use them if you need to debug, otherwise it is advisable to keep them disable.
	- `DEBUG_PRINTF`: if defined, activates the debug printf in the code. Works only if `PRINTF_ACTIVE` is defined. Used to separate debug messages from other type of printf, thus to active only the required group to not overburden the PCF execution with several printf()
	- `HRO_PRINTF`: if defined, activates "messages for humans" printf in the code. Works only if `PRINTF_ACTIVE` is defined. Used to separate debug messages from other type of printf, thus to active only the required group to not overburden the PCF execution with several printf()
	- `CTRL_DEBUG`: activate several printf along the code that print the output of each step of the control algorithm. May work independently of `PRINTF_ACTIVE` but it is advised to define it.
	- `PCF_USE_CLUSTER`: if defined, activate the cluster (both the hardware and the software/control part)
	- `PCF_USE_CLUSTER_PARALL`: if defined the control is split into all the cluster core to obtain parallel execution. Needs `PCF_USE_CLUSTER` to be defined.
	- ~~`TEST_GSL_LIBRARY`: legacy parameter to use GLS library for Newton Rapson.~~ no longer used in the code
	- `GSL_PARALLEL`: if defined, the parallel version of Newton-Rapson for the cluster is used.

Other parameters instead, require to be defined to specific values:
- Firmware:
	- `DEFAULT_SYSTEM_CLOCK=20000000u`: define the hardware clock. Not sure if it has an impact on changing hardware clock, but it is needed for some computation inside the PCF. 20000000u is the default
	- `PCF_FREERTOS_TICK_RATE_HZ`: define the tick rate in HZ of FreeRTOS. Default FreeRTOS value is 1000. **NOTE** consider the simulation slowdown and change this parameter accordingly. For example, for a 25x slowdown, this value should be set to 40
	- `TASKS_PERIOD_MULTIPLIER`: used to set the slowdown of the simulation. With a 25x simulation slowdown, this can be set to 25. **NOTE** Due to internal PCF computation, this value should not be too high (I would say no more than 100, but that may also depends by the periods of all the tasks of the PCF). To go higher, use the `TASKS_PERIOD_MULTIPLIER_P2`
	- `TASKS_PERIOD_MULTIPLIER_P2`: used as a multiplicative factor to the `TASKS_PERIOD_MULTIPLIER`, to reach to higher slowdown factors. E.g. 1000x slowdown can be recast as 25 `TASKS_PERIOD_MULTIPLIER` and 4 TASKS_PERIOD_MULTIPLIER_P2
	- `CONTROL_DATA_TYPE`: this variable indicate the type of variable for the control data and function. This translate into different `varValue` types. Currently only the value `1` for `float` is working

In the Makefile there is also the variable (not to be confused with the code variables seen before) `SCMI` that, if defined to 1 will activate the **SCMI** feature. ATM is detached from most of the other.

**Additional definitions and parameters** that could be **important** for the PCF execution are found:
 - in the `Config/ folder`
 - in the file `PCF_Core/Secure/sec_globalProtectedVar.c`
 these, for example, may be the number of cores, the number of voltage domains, the division of cores in the domains, etc.

### Debug of Firmware execution
There is the possibility of doing 2 type of tests: one measure the timing registered from the clock counting registers, and the other is a sort of automatic execution test, internally called "CI". For the measures (which the main point was to measure time, but that can be also used to measure other type of metric, such as stalls, memory transaction, instructions retired, etc.), the parameters to define are:
 - `MEASURE_ACTIVE`: a test using performance counters. (=1 to print performances, other tests in the future). If defined it activates the test.
 - `MEASURE_N_ITERATION`: the number of measurements **iterations** (see Measure char meanings below);
 - `MEASURE_N_OUTPUTS` indicates the number of measurements present in the code per iteration, default is 32.
 - `MEASURE_CSR` if you use the general CSR-Read function `lMeasureReadCsr()` it indicates the **offset** from the base address setted in the measure.c file in /Target/Debug. It is target dependent (which is not elegant) but atm is the only feasible solution.
 - `MEASURE_ZEROING` if =1 the counting read register will be reset on leave, if =0 it won't. It is NOT working atm.
 
For the "CI" test are:
 - `CI_TEST`: this is to activate the CI test. Currently `CI_TEST` and `MEASURE_ACTIVE` cannot be active at the same time
 - `CI_TEST_ITERATION`: the number of iteration for the test.


### Scripts
There are already several compilation scripts in the compile_scripts folder.
**TO BE NOTED:** only the compile_fpga.sh is kept up-to-date.


## Rules for coding

### MAIN Rules:
* Keeps the core of the firmware implementation-free, meaning it can be implemented in every possible situation and processor.

### Names:
* __Files:__ `(folder abbr)_name.h/c` all lowercase, example: `cfg_system.h`


 Folder | Abbr
 ------------- | -------------
 main | pcf
 configuration | cfg
 control | ctrl
 implementation | imp
 target | tgt
 * __Func:__ `(return)(Category/Group/Folder Name)(Func Name)`  (return) lowercase, (other) is first UpperCase, all linked,  example: `ulCommsInit();`


 Type | Abbr
 ------------- | -------------
void | v
Integer | l
short | s
struct | x
varValue | ?
unsigned | u(+other)
bool | b

* __Var:__ `(type)_name_of_the_var` all lowercase, separated by underscore \_, at beginning a type,  example: `g_config_sys`, `number_of_slots`


Type | Abbr
 ------------- | -------------
global | g
static | l
func input | i
func output | o
other | (empty)

* __Typedef:__
 * Type: `varName` var + name of type of variable with first UpperCase, all linked, example: `varValue`
 * Struct: `name_t`
 * enum: `name_e`
* __Macro/Define:__ all CAPS; `VD` = value definition (e.g. `VD_ZERO` = 0)

### Varius:
* __num rule:__ names with num should be all this way: `num_something`, `max_num_something`
* __use rule:__ bool variables (mostly for config) should be named all `use_name`
* __config\_table:__ constant, unchangeable, default, initial values
* __parameter\_table:__ changeable, passed through agents, to communicate stuff.
* __TODO:__ To do, a thing that is left unfinished and should be done before the release
* __TBC:__ To be checked, a thing that is done/implemented, but needs a second opinion from someone else
* __TBD:__ To be defined, a thing that is implemented, but needs Approval
* __TBU:__ To be updated, a thing that is left rough (but completely working), waiting for ideas or time to change it
* __RTDO:__ Robert Discussion + things to change

## Measure char meanings:
```
    I: Interrupt
    K: Systick

    -------------- Periodic Task
    P: Task Awakened/start
    s: Send Frequency end
    T: Telemetry computation end (telemetry is not always executed)
    r: Receive Temperature end
    C: Control part
    p: Task end
    -- Time between C and p is dedicated to Shared variable writing

    -------------- TaskOS
    O: Task Awakened/start
    -- Time Between O and A is needed to read shared variables
    A: parameter adaptation
    o: Task end.
    -- Time between A and o is used to write global var.

    -------------- CommsTask
    M: Task Awakened/start
    m: task end  

    -------------- MainTask
    G: Task Awakened/start
    g: task end 

    -------------- Fast Control Task
    F: Task Awakened/start
    f: task end 

    -------------- User-Defined Task
    U: Task Awakened/start
    u: task end 

```

## pmsis_exit() Code Values:

Code Values | Meanings
------------- | -------------
 -10 | Sem Init Failure
 -9 | exited the for infinite loop of the task
 -5 | error in task creation
 -4 | error in FreeRTOS init
 -3 | error in initial parameters
 -2 | error in hardware initialization
 -1 | FATAL
 0 | succeded test
 1 | failed test

