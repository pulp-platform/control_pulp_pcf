GSL: ./configure --host=riscv32-unknown-elf --build=x86_64-pc-linux-gnu --target=riscv32-unknown-elf --prefix=/usr/pack/riscv-1.0-kgf/pulp-gcc-2.5.0-rc1/bin/ CFLAGS="-march=rv32imc_zfinx_xcorev -mabi=ilp32"

# PCF PROJECT
## AN OPEN-SOURCE POWER AND TEMPERATURE CONTROL FIRMWARE

This project is an open-source Power and Temperature firmware for Multi-Core
CPUs, which aim to be scalable and exportable to many chips and platforms.

The full documentation can be found here: *currently in development*.

## Directory Structure
```
├── PCF_Core            Contains all the code of the Firmware
    ├── Config          Contains the configuration headers and the #define to modify the code
    ├── Control         Contains all the Control part and the Math libraries
    ├── Secure          This is a "secure environment" with functions and tasks that have
                            high impact on security
    ├── Target          Contains the functions that are Target-dependent.
        ├── DEBUG       Contains files to help the debug and the auto-tests
├── System_Impl         Contains the functions that are System/Platform-dependent
FreeRTOSConfig.h        File needed for FreeRTOS
MakeFile                The Makefile
├── Others              A folder containing other files
```

## To Compile

To look for specific commands depending on your setup, see below the various sections.

--

* The variable **PCF_PLATFORM** has to be defined as the name of the folder of the System
implementation: e.g. `export PCF_PLATFORM=FPGA_ZCU102` . Default value is NO_IO.
* The variable **PCF_TARGET** has to be defined as the name of the folder of the Target
implementation: e.g. `export PCF_TARGET=PMS_RI5CY` . Default value is PMS_CV32.

Other definition values to check (they can be also defined inside PCF_Core/Config/cfg_firmware.h:)
+ `MEASURE_ACTIVE`: a test using performance counters. (=1 to print performances, other tests in the future)
    * activate a timing test;
    * gives you `MEASURE_N_ITERATION` measurements **iterations** (see Measure char meanings below);
    * `MEASURE_N_OUTPUTS` indicates the number of measurements present in the code per iteration, default is 32.
    * `MEASURE_CSR` if you use the general CSR-Read function `lMeasureReadCsr()` it indicates the **offset** from the base address setted in the measure.c file in /Target/Debug. It is target dependent (which is not elegant) but atm is the only feasible solution.
    * `MEASURE_ZEROING` if =1 the counting read register will be reset on leave, if =0 it
    won't. It is NOT active atm.
+ `CI_TEST`: this is to activate the CI test. Currently `CI_TEST` and `MEASURE_ACTIVE` cannot be active at the same time.
 In the future it will be implemented differently.
+ `CONTROL_DATA_TYPE`: it defines the type of data to be used in the control part of the PCF firmware. The file that will refer to this
option is PCF_Core/Config/cfg_types.h .
    * =1 to use float;
    * =2 to use float16; (not supported atm)
    * =3 to use fixed int; (not supported atm)
    * other values, or if left not defined, will result in int (which is not supported atm).
+ `PRINTF_ACTIVE`: it is used to enable printf all around the code to print errors and
issues during the execution.

### For CI:

Assuming RISCV_TOOLCHAIN and FREERTOS_PROJ_ROOT are well-set, the variables PCF_TARGET and
PCF_PLATFORM doesn't need to be set since the defaults are tailored for CI.
The command is:

```
make MAKE_FLAGS="-DCI_TEST_ITERATION=100 -UMEASURE_ACTIVE -DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=2000 -DDEFAULT_SYSTEM_CLOCK=100000000u -DCI_TEST -DCONTROL_DATA_TYPE=1" all
```

Of course you can set in `-DCI_TEST_ITERATION=` the value you prefer. Higher values holds longer
testing times. The minimum value should be at least 5.

### For DATE paper:

Assuming RISCV_TOOLCHAIN and FREERTOS_PROJ_ROOT are well-set, the command is:

```
export PCF_TARGET=PMS_CV32
export PCF_PLATFORM=DATE_PAPER
make MAKE_FLAGS="-DPRINTF_ACTIVE -DPCF_FREERTOS_TICK_RATE_HZ=2000 -DDEFAULT_SYSTEM_CLOCK=100000000u -DCONTROL_DATA_TYPE=1" clean all run gui=1
```

If you desire to terminate the test early (the complete simulation last about 24h) you could add either `-DCI_TEST -DCI_TEST_ITERATION=N -DMEASURE_ACTIVE=2` or `-DMEASURE_ACTIVE=1 -DMEASURE_N_ITERATION=N`, with N= the number of iterations you would like (consider that N is preferable to be even, and that 2 iterations in the Modelsim simulation last about 40s). With `MEASURE_ACTIVE=1` at the end of the firmware execution, about `32 x N` printf() will be casted, and that could slow down a bit the simulation time, but also you will receive data from the Performance Counters regarding the simulated workflow (i.e. the firmware). You can also add some "*measurement points*" in the code wherever you like by including `"measure.h"` and using the command:
```
timerBuffer[Timerindex++] = (Timer_Data_t) {'c', lMeasureReadCsr(MEASURE_ZEROING)};
```
with instead of '*c*' add the char character you prefer to identify that point. Just remember to also change `MEASURE_N_OUTPUTS` accordingly by adding `-DMEASURE_N_OUTPUTS=K` with `K=32+L` being L= the number of "*points of measure*" you added.


### For General Implementation:

```
export RISCV_TOOLCHAIN=<your_riscv_toolchain_path>
export FREERTOS_PROJ_ROOT=<your_freertos_path>
source $FREERTOS_PROJ_ROOT/env/control-pulp.sh
export PCF_TARGET=<name_of_the_folder_under_./PCF_Core/Target/)>
export PCF_PLATFORM=<name_of_the_folder_under_./System_Implementation)>

make MAKE_FLAGS="-U<the_config_you_want_to_change> -D<the_config_you_want_to_change>" all
```
The configuration definitions can be done directly in the command with the MAKE_FLAGS, or also
directly inside the Makefile with the CPPFLAGS.
Ultimately one option could be also in the configuration file but you have to remove
all -D definitions from the Makefile and the command.

The default choice for PCF_PLATFORM and PCF_TARGET is:

```
export PCF_TARGET=PMS_CV32
export PCF_PLATFORM=NO_IO
```
also, not defining them (so no export command launched before compilation) will arrive
to the same result.


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



## Other stuff to remember
`JO_TBR`
Structure:
    default_* contains all default values for configuration global var
    there are n (5) global config var
    there are p (2) global parameter var
