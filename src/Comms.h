/**
 *
 */

#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include "Packet.h"

#define SEND_NUM_RETRIES 5

/**
 *
 */
//int sendPacket(Packet* packet);

/**
 *
 */
bool isACK(uint8_t* p);

#endif /* COMMS_H */
