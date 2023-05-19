
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




#include "imp_dataLib.h"
#include "cfg_types.h"
#include "cfg_firmware.h"
#include "cfg_system.h"


int no_io_uiIndex = 0;
int no_io_uiCounter = 0;
int no_io_periodicIndex = 0;
int no_io_periodicCounter = 0;


/************ DATA WILL BE OUTPUTTED HERE **************/

//varValue no_io_uiFreqRedMap[NO_IO_NUMBER_OF_COMMANDS * NO_IO_UI_STEPS] = {{0}};
//varValue no_io_uiErrorMap[NO_IO_NUMBER_OF_COMMANDS * NO_IO_UI_STEPS]  = {{0}};

varValue no_io_computedFreq[NO_IO_PERIODIC_TOT_STEPS][MAX_NUM_CORE]  = {{0}};
//actually these values above are a bit different because of approximation
//in fact to pass the data thorugh the spi I pass it as a short unsigned int
//so I make at the 4th decimal. BTW in the new implementation this should not
//be a problem since I stepped the freq with 100Mhz steps---

#ifdef NO_IO_FREQ_COMPARE
varValue no_io_freqDiff[NO_IO_PERIODIC_TOT_STEPS][MAX_NUM_CORE] = {{0}};
#endif

/*************** INSERT HERE THE DATA *****************/

//The temperature measured by the sensor and received by the fimware. One temp for each core for each step
varValue no_io_measuredTemp[NO_IO_PERIODIC_TOT_STEPS][MAX_NUM_CORE] = { {2.532888e+01f, 2.532913e+01f, 2.532858e+01f, 2.532449e+01f, 2.532449e+01f, 2.531991e+01f, 2.531961e+01f, 2.531524e+01f, 2.531500e+01f, 2.532888e+01f, 2.532913e+01f, 2.532858e+01f, 2.532449e+01f, 2.532449e+01f, 2.531991e+01f, 2.531961e+01f, 2.531524e+01f, 2.531500e+01f, 2.532888e+01f, 2.532913e+01f, 2.532858e+01f, 2.532449e+01f, 2.532449e+01f, 2.531991e+01f, 2.531961e+01f, 2.531524e+01f, 2.531500e+01f, 2.532888e+01f, 2.532913e+01f, 2.532858e+01f, 2.532449e+01f, 2.532449e+01f, 2.531991e+01f, 2.531961e+01f, 2.531524e+01f, 2.531500e+01f},
{4.973016e+01f, 4.971166e+01f, 4.971884e+01f, 4.972198e+01f, 4.971008e+01f, 4.971042e+01f, 4.973727e+01f, 4.972180e+01f, 4.972717e+01f, 4.973016e+01f, 4.971166e+01f, 4.971884e+01f, 4.972198e+01f, 4.971008e+01f, 4.971042e+01f, 4.973727e+01f, 4.972180e+01f, 4.972717e+01f, 4.973016e+01f, 4.971166e+01f, 4.971884e+01f, 4.972198e+01f, 4.971008e+01f, 4.971042e+01f, 4.973727e+01f, 4.972180e+01f, 4.972717e+01f, 4.973016e+01f, 4.971166e+01f, 4.971884e+01f, 4.972198e+01f, 4.971008e+01f, 4.971042e+01f, 4.973727e+01f, 4.972180e+01f, 4.972717e+01f},
{5.218249e+01f, 5.217464e+01f, 5.217434e+01f, 5.216057e+01f, 5.215368e+01f, 5.215051e+01f, 5.214281e+01f, 5.213339e+01f, 5.213927e+01f, 5.218249e+01f, 5.217464e+01f, 5.217434e+01f, 5.216057e+01f, 5.215368e+01f, 5.215051e+01f, 5.214281e+01f, 5.213339e+01f, 5.213927e+01f, 5.218249e+01f, 5.217464e+01f, 5.217434e+01f, 5.216057e+01f, 5.215368e+01f, 5.215051e+01f, 5.214281e+01f, 5.213339e+01f, 5.213927e+01f, 5.218249e+01f, 5.217464e+01f, 5.217434e+01f, 5.216057e+01f, 5.215368e+01f, 5.215051e+01f, 5.214281e+01f, 5.213339e+01f, 5.213927e+01f},
{7.701581e+01f, 7.704516e+01f, 7.728158e+01f, 7.729394e+01f, 7.719479e+01f, 7.721652e+01f, 7.717242e+01f, 7.692800e+01f, 7.692406e+01f, 7.701581e+01f, 7.704516e+01f, 7.728158e+01f, 7.729394e+01f, 7.719479e+01f, 7.721652e+01f, 7.717242e+01f, 7.692800e+01f, 7.692406e+01f, 7.701581e+01f, 7.704516e+01f, 7.728158e+01f, 7.729394e+01f, 7.719479e+01f, 7.721652e+01f, 7.717242e+01f, 7.692800e+01f, 7.692406e+01f, 7.701581e+01f, 7.704516e+01f, 7.728158e+01f, 7.729394e+01f, 7.719479e+01f, 7.721652e+01f, 7.717242e+01f, 7.692800e+01f, 7.692406e+01f},
{7.695257e+01f, 7.697851e+01f, 7.706539e+01f, 7.705883e+01f, 7.700656e+01f, 7.701208e+01f, 7.699990e+01f, 7.688223e+01f, 7.687005e+01f, 7.695257e+01f, 7.697851e+01f, 7.706539e+01f, 7.705883e+01f, 7.700656e+01f, 7.701208e+01f, 7.699990e+01f, 7.688223e+01f, 7.687005e+01f, 7.695257e+01f, 7.697851e+01f, 7.706539e+01f, 7.705883e+01f, 7.700656e+01f, 7.701208e+01f, 7.699990e+01f, 7.688223e+01f, 7.687005e+01f, 7.695257e+01f, 7.697851e+01f, 7.706539e+01f, 7.705883e+01f, 7.700656e+01f, 7.701208e+01f, 7.699990e+01f, 7.688223e+01f, 7.687005e+01f},
{7.689410e+01f, 7.711312e+01f, 7.695141e+01f, 7.692843e+01f, 7.690441e+01f, 7.689303e+01f, 7.710217e+01f, 7.703491e+01f, 7.702358e+01f, 7.689410e+01f, 7.711312e+01f, 7.695141e+01f, 7.692843e+01f, 7.690441e+01f, 7.689303e+01f, 7.710217e+01f, 7.703491e+01f, 7.702358e+01f, 7.689410e+01f, 7.711312e+01f, 7.695141e+01f, 7.692843e+01f, 7.690441e+01f, 7.689303e+01f, 7.710217e+01f, 7.703491e+01f, 7.702358e+01f, 7.689410e+01f, 7.711312e+01f, 7.695141e+01f, 7.692843e+01f, 7.690441e+01f, 7.689303e+01f, 7.710217e+01f, 7.703491e+01f, 7.702358e+01f},
{7.683972e+01f, 7.711782e+01f, 7.707461e+01f, 7.683681e+01f, 7.683214e+01f, 7.681310e+01f, 7.690011e+01f, 7.685628e+01f, 7.684274e+01f, 7.683972e+01f, 7.711782e+01f, 7.707461e+01f, 7.683681e+01f, 7.683214e+01f, 7.681310e+01f, 7.690011e+01f, 7.685628e+01f, 7.684274e+01f, 7.683972e+01f, 7.711782e+01f, 7.707461e+01f, 7.683681e+01f, 7.683214e+01f, 7.681310e+01f, 7.690011e+01f, 7.685628e+01f, 7.684274e+01f, 7.683972e+01f, 7.711782e+01f, 7.707461e+01f, 7.683681e+01f, 7.683214e+01f, 7.681310e+01f, 7.690011e+01f, 7.685628e+01f, 7.684274e+01f} };


//The taget frequency per core (input) for each different input command
varValue no_io_uiTargetFreq[NO_IO_NUMBER_OF_COMMANDS][MAX_NUM_CORE] = { {2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f}, //0
                            {2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f, 2.5000f},
                            {1.9000f, 2.2000f, 2.2000f, 1.9000f, 2.5000f, 1.9000f, 1.9000f, 2.2000f, 2.3000f, 1.9000f, 2.2000f, 2.2000f, 1.9000f, 2.5000f, 1.9000f, 1.9000f, 2.2000f, 2.3000f, 1.9000f, 2.2000f, 2.2000f, 1.9000f, 2.5000f, 1.9000f, 1.9000f, 2.2000f, 2.3000f, 1.9000f, 2.2000f, 2.2000f, 1.9000f, 2.5000f, 1.9000f, 1.9000f, 2.2000f, 2.3000f},
							{1.9000f, 2.2000f, 2.2000f, 1.9000f, 2.5000f, 1.9000f, 1.9000f, 2.2000f, 2.3000f, 1.9000f, 2.2000f, 2.2000f, 1.9000f, 2.5000f, 1.9000f, 1.9000f, 2.2000f, 2.3000f, 1.9000f, 2.2000f, 2.2000f, 1.9000f, 2.5000f, 1.9000f, 1.9000f, 2.2000f, 2.3000f, 1.9000f, 2.2000f, 2.2000f, 1.9000f, 2.5000f, 1.9000f, 1.9000f, 2.2000f, 2.3000f} }; //Insert here!

//The target workload per core (input) for each different input command
varValue no_io_workload[NO_IO_NUMBER_OF_COMMANDS][MAX_NUM_CORE] = { {0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f, 0.5333f}, //0
									{1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f, 1.2444f},
									{1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f, 1.7778f},
									{1.2444f, 1.7778f, 1.2444f, 0.5333f, 1.2444f, 1.7778f, 1.2444f, 0.5333f, 1.2444f, 1.2444f, 1.7778f, 1.2444f, 0.5333f, 1.2444f, 1.7778f, 1.2444f, 0.5333f, 1.2444f, 1.2444f, 1.7778f, 1.2444f, 0.5333f, 1.2444f, 1.7778f, 1.2444f, 0.5333f, 1.2444f, 1.2444f, 1.7778f, 1.2444f, 0.5333f, 1.2444f, 1.7778f, 1.2444f, 0.5333f, 1.2444f} }; //Insert here!

//The target power budget (uniquef, not per core) for each different input command
varValue no_io_uiPowerBudget[NO_IO_NUMBER_OF_COMMANDS] = { 90.0f, 90.0f, 50.0f, 90.0f }; //Insert here!

//The actual recorded total power consumed by the model - If you do not have the data just put 0f, it should be fine for now
varValue no_io_simPower[NO_IO_PERIODIC_TOT_STEPS] = { //[NO_IO_NUMBER_OF_COMMANDS * NO_IO_UI_STEPS] = {
5.817125e+00f,
5.817125e+00f,
9.950851e+00f,
1.585575e+01f,
1.021303e+01f,
5.817125e+00f,
5.817125e+00f };

#ifdef NO_IO_FREQ_COMPARE
//If you want to comparef, insert here the data of the Outputted freq per core for each step.
varValue no_io_recordedFreqOut[NO_IO_PERIODIC_TOT_STEPS][MAX_NUM_CORE] = {{1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f, 1.500000e+00f},
{2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f},
{2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f},
{2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f},
{2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f},
{2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f},
{2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f, 2.000000e+00f, 1.700000e+00f, 1.700000e+00f, 2.500000e+00f, 2.299900e+00f, 1.799900e+00f, 2.299900e+00f, 2.500000e+00f, 2.099900e+00f} };
#endif
