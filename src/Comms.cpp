/**
 *
 */

#include <Arduino.h>
#include "Comms.h"
#include "Packet.h"

uint8_t PACKET_HEADER[3] = { 0x50, 0x50, 0x50 };
uint8_t ACK[3] = { 0xAA, 0x05, 0x00 };
uint8_t NAK[3] = { 0xAA, 0x05, 0xFF };

//int sendPacket(Packet* packet) {}

bool isACK(uint8_t* p) {
    return p[2] == 0;
}
