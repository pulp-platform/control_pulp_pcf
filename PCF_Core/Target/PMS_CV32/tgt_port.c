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

/* FreeRTOS Inclusions. */
#include <FreeRTOS.h>
#include <task.h>

#include "tgt_port.h"

/* Target Includes */
#include "system.h"
#include "os.h"

/* Libraries Inclusion */
#include "cfg_types.h"
#include "cfg_constants.h"


void vTargetPcfExit(int exit_value)
{
	//TODO: if scheduler running:
	//vTaskSuspendAll();

	pmsis_exit(exit_value);

	//if i put this after it's ok, no?
	vTaskSuspendAll();
}
