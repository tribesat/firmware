/**
 *
 */

#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include "Packet.h"

#define SEND_NUM_RETRIES 5

extern uint8_t PACKET_HEADER[3];
extern uint8_t ACK[3];
extern uint8_t NAK[3];

/**
 *
 */
//int sendPacket(Packet* packet);

/**
 *
 */
bool isACK(uint8_t* p);

#endif /* COMMS_H */
