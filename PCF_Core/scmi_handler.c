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

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

/* system includes */
#include "system.h"
#include "io.h"
#include "irq.h"
#include "csr.h"
#include "clic.h"
#include "scmi.h"

/* pmsis */
// #include "target.h"
// #include "os.h"

#include "scmi_handler.h"

uint32_t scmi_target_frequency = 0x0;
// TODO:
uint32_t scmi_target_changed = 0;

void (*clic_isr_hook[1])(void);

void exit_success(void) {
    // printf("someone wrote to mailbox!");
    callee_scmi_handler();
}

void complete_msg(uint32_t header, uint32_t agent_id) {
    // write the header back
    writew(header, MBOX_START_ADDRESS + SCMI_MESSAGE_HEADER_C0_REG_OFFSET);

    // write channel status to free
    writew(0x0, MBOX_START_ADDRESS + SCMI_CHANNEL_STATUS_C0_REG_OFFSET);

    // ring the completion
    writew(agent_id, MBOX_START_ADDRESS + SCMI_COMPLETION_INTERRUPT_C0_REG_OFFSET);
}

void scmi_init(void) {
    // // printf("test csr accesses\n");
    // uint32_t thresh = 0xffaa;
    // uint32_t cmp = 0;
    // csr_write(CSR_MINTTHRESH, thresh);
    // cmp = csr_read(CSR_MINTTHRESH);
    // csr_write(CSR_MINTTHRESH, 0);	/* reset threshold */
    // assert(cmp == (thresh & 0xff)); /* only lower 8 bits are writable */

    // /* redirect vector table to our custom one */
    // // printf("set up vector table\n");
    // clic_setup_mtvec();
    // clic_setup_mtvt();

    // /* enable selective hardware vectoring */
    // // printf("set shv\n");
    // writew((0x1 << CLIC_CLICINTATTR_SHV_BIT),
    //        CLIC_BASE_ADDR + CLIC_CLICINTATTR_REG_OFFSET(INTR_ID));

    // /* set trigger type to edge-triggered */
    // // printf("set trigger type: edge-triggered\n");
    // writeb((0x1 << CLIC_CLICINTATTR_TRIG_OFFSET) |
    // 	       readw(CLIC_BASE_ADDR +
    // 		     CLIC_CLICINTATTR_REG_OFFSET(INTR_ID)),
    //        CLIC_BASE_ADDR + CLIC_CLICINTATTR_REG_OFFSET(INTR_ID));

    // /* set number of bits for level encoding:
    //  * nlbits
    //  */
    // // printf("set nlbits\n");
    // writeb((0x4 << CLIC_CLICCFG_NLBITS_OFFSET),
    //        CLIC_BASE_ADDR + CLIC_CLICCFG_REG_OFFSET);

    // /* set interrupt level and priority*/
    // // printf("set interrupt priority and level\n");
    // writew(0xaa, CLIC_BASE_ADDR + CLIC_CLICINTCTL_REG_OFFSET(INTR_ID));

    // /* raise interrupt threshold to max and check that the interrupt doesn't
    //  * fire yet */
    // // printf("raise interrupt threshold to max (no interrupt should happen)\n");
    // //csr_write(CSR_MINTTHRESH, 0xff); /* 0xff > 0xaa */
    // //clic_isr_hook[0] = exit_fail; /* if we take an interrupt then we failed */

    // // printf("lower interrupt threshold\n");
    // clic_isr_hook[0] = exit_success;
    // csr_write(CSR_MINTTHRESH, 0); /* 0 < 0xaa */

    /* raise interrupt threshold to max and check that the interrupt doesn't
     * fire yet */
    // printf("raise interrupt threshold to max (no interrupt should happen)\n");
    csr_write(CSR_MINTTHRESH, 0xff); /* 0xff > 0xaa */
    irq_set_handler(INTR_ID, callee_scmi_handler);
    irq_enable(INTR_ID);
    irq_clint_enable(INTR_ID);
}

void enable_scmi_interrupt(void) {
    /* enable interrupt globally */
    // irq_clint_global_enable();

    /* enable interrupt on clic */
    // writew(0x1, CLIC_BASE_ADDR + CLIC_CLICINTIE_REG_OFFSET(INTR_ID));

    pulp_irq_init();
}

void callee_scmi_handler(void) {

    uint32_t agent_id = 0x0;
    uint32_t data = 0x0;
    uint32_t payload = 0x0;
    uint32_t response = 0x0;
    uint32_t protocol_id = 0x0;
    uint32_t message_id = 0x0;

    // read agent_id from doorbell
    agent_id = readw(MBOX_START_ADDRESS + SCMI_DOORBELL_C0_REG_OFFSET);

    // read payload
    payload = readw(MBOX_START_ADDRESS + SCMI_MESSAGE_PAYLOAD_0_C0_REG_OFFSET);
    // printf("payload %u\n\r", payload);
    // printf("payload +4 %u\n\r", readw(MBOX_START_ADDRESS +
    // SCMI_MESSAGE_PAYLOAD_0_C0_REG_OFFSET+4));
    // printf("payload +8 %u\n\r", readw(MBOX_START_ADDRESS +
    // SCMI_MESSAGE_PAYLOAD_0_C0_REG_OFFSET+8));

    // read header
    data = readw(MBOX_START_ADDRESS + SCMI_MESSAGE_HEADER_C0_REG_OFFSET);
    protocol_id = (data & 0x3fc00) >> 10;
    message_id = data & 0xff;

    // clear doorbell
    writew(0x0, MBOX_START_ADDRESS + SCMI_DOORBELL_C0_REG_OFFSET);

    if (protocol_id == 0x13) {
        // printf("msgid: %x\n\r", message_id);
        if (message_id == 0x00) { // protocol version
            response = SUCCESS;
            writew(response, MBOX_START_ADDRESS + SCMI_MESSAGE_PAYLOAD_0_C0_REG_OFFSET);

            response = 0x30000;
            writew(response, MBOX_START_ADDRESS + SCMI_MESSAGE_PAYLOAD_0_C0_REG_OFFSET + 4);

            writew(8 + 4 * 1, MBOX_START_ADDRESS + SCMI_LENGTH_C0_REG_OFFSET);

            complete_msg(data, agent_id);
        } else if (message_id == 0x07) { // perf level set
            response = SUCCESS;
            writew(response, MBOX_START_ADDRESS + SCMI_MESSAGE_PAYLOAD_0_C0_REG_OFFSET);

            writew(8 + 4 * 0, MBOX_START_ADDRESS + SCMI_LENGTH_C0_REG_OFFSET);

            if (scmi_target_frequency != payload) {
                scmi_target_changed = 1;
            }
            scmi_target_frequency = payload;

            complete_msg(data, agent_id);
        } else if (message_id == 0x08) {
            response = SUCCESS;
            writew(response, MBOX_START_ADDRESS + SCMI_MESSAGE_PAYLOAD_0_C0_REG_OFFSET);

            response = scmi_target_frequency; // current perf level
            writew(response, MBOX_START_ADDRESS + SCMI_MESSAGE_PAYLOAD_0_C0_REG_OFFSET + 4);

            writew(8 + 4 * 1, MBOX_START_ADDRESS + SCMI_LENGTH_C0_REG_OFFSET);

            complete_msg(data, agent_id);
        } else {
            printf("[SCMI] Wrong message id!\n\r");
        }
    } else {
        printf("[SCMI] Wrong protocol id!\n\r");
    }
}
