/**
 * Packet structure:
 *   Temperature: 0 - 1
 */
#define DEBUG // TODO: REMOVE ME

#include <Arduino.h>
#include "Packet.h"

/**
 * Constructor
 */
Packet::Packet() {
    this->header = this->packet; // header points to the top of the packet
    this->data = this->packet + PACKET_HEADER_SIZE; // data is offset PACKET_HEADER_SIZE bytes

    // set header 0x50, 0x50 0x50
    this->header[0] = this->header[1] = this->header[2] = 0x50;
}

/**
 * Destructor
 */
Packet::~Packet() {}

#ifdef DEBUG
/**
 * Prints the packet to Serial
 */
void Packet::print() {
    Serial.print("[Header = ");
    for (int i = 0; i < PACKET_HEADER_SIZE; i++) {
        Serial.print(this->header[i], HEX);
        Serial.print(" ");
    }

    Serial.print(", Data = ");
    for (int i = 0; i < PACKET_DATA_SIZE; i++) {
        Serial.print(this->data[i], HEX);
        Serial.print(" ");
    }

    Serial.println("]");

    Serial.print("Temperature: ");
    Serial.println(*(int*)this->data);
}
#endif /* DEBUG */

/**
 * Encode sensor data into the packet
 */
void Packet::setField(PacketField field, void* data) {
    int *d;
    switch (field) {
        case Temperature:
            d = (int*)this->data;
            *d = *((int*)data);
            break;
#ifdef DEBUG
        default:
            Serial.println("PACKET: invalid packet field");
#endif /* DEBUG */
    }
}

/**
 * Returns a pointer to the underlying array that is the packet
 */
uint8_t* Packet::toArray() {
    return this->packet;
}
