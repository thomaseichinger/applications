/*
 * Copyright (C) 2015 Thomas Eichinger
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @file
 * @brief       Example application for Embedded World 2015
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 */

#include <stdio.h>
#include <string.h>

#include "vtimer.h"
#include "srf02.h"
#include "periph/i2c.h"

#include "thread.h"

#include "ieee802154_frame.h"
#include "transceiver.h"

#define SND_BUFFER_SIZE     (100)
#define RCV_BUFFER_SIZE     (64)
#define RADIO_STACK_SIZE    (KERNEL_CONF_STACKSIZE_DEFAULT)

#define SENSOR_NODE_DISTANCE_TYPE 0x55

static msg_t msg_q[RCV_BUFFER_SIZE];

#define SLEEP       (1000 * 1000U)

static srf02_t srf02_0;
static char text_msg[20];


void init_transceiver(void)
{
    uint16_t transceivers = TRANSCEIVER_DEFAULT;

    transceiver_init(transceivers);
    (void) transceiver_start();
    transceiver_register(transceivers, thread_getpid());
}

void transceiver_send_handler(uint16_t dest)
{
    if (transceiver_pid == KERNEL_PID_UNDEF) {
        puts("Transceiver not initialized");
        return;
    }

    ieee802154_packet_t p;

    transceiver_command_t tcmd;
    tcmd.transceivers = TRANSCEIVER_AT86RF231;
    tcmd.data = &p;

    memset(&p, 0, sizeof(ieee802154_packet_t));
    p.frame.payload = (uint8_t*) text_msg;
    p.frame.payload_len = strlen(text_msg) + 1;
    p.frame.fcf.frame_type = IEEE_802154_DATA_FRAME;
    p.frame.fcf.dest_addr_m = IEEE_802154_SHORT_ADDR_M;
    p.frame.fcf.src_addr_m = IEEE_802154_SHORT_ADDR_M;
    p.frame.dest_addr[1] = (dest&0xff);
    p.frame.dest_addr[0] = (dest>>8);
    p.frame.dest_pan_id = SENSOR_NODE_PAN_ID;

    msg_t mesg;
    mesg.type = SND_PKT;
    mesg.content.ptr = (char *) &tcmd;

    printf("[transceiver] Sending packet of length %" PRIu16 " to %" PRIu16 ": %s\n", p.frame.payload_len, p.frame.dest_addr[1], (char*) p.frame.payload);

    msg_send_receive(&mesg, &mesg, transceiver_pid);
    int8_t response = mesg.content.value;
    printf("[transceiver] Packet sent: %" PRIi8 "\n", response);
}

int main(void)
{
    int res;
    msg_t m;

    printf("Initializing SRF02 sensor at I2C_%i... ", SRF02_I2C);
    res = srf02_init(&srf02_0, SRF02_I2C, SRF02_DEFAULT_ADDR, SRF02_SPEED);

    if (res < 0) {
        printf("[Failed]");
        return 1;
    }
    puts("[Ok]\n");

    init_transceiver();


    msg_init_queue(msg_q, RCV_BUFFER_SIZE);

    memset(text_msg, 0xaa, 20);
    text_msg[0] = SENSOR_NODE_SRC_ID;
    text_msg[1] = SENSOR_NODE_DISTANCE_TYPE;

    while(1) {
        msg_try_receive(&m);
        if (m.type == PKT_PENDING) {
            ((ieee802154_packet_t*) m.content.ptr)->processing--;

        }
        uint16_t distance = srf02_get_distance(&srf02_0, SRF02_MODE);
        printf("distance: %x\n", distance);
        text_msg[2] = (distance>>8);
        text_msg[3] = (char)(distance&0xff);
        transceiver_send_handler(CENTRAL_NODE_ADDR);        

        vtimer_usleep(SLEEP);
    }
}