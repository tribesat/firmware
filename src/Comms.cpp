/**
 *
 */

#include <Arduino.h>
#include "Comms.h"
#include "Packet.h"

//int sendPacket(Packet* packet) {}

bool isACK(uint8_t* p) {
    return p[2] == 0;
}
