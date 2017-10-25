/**
 *
 */
#define DEBUG // TODO: REMOVE ME

#include <Arduino.h>
#include "Packet.h"

/**
 * Constructor with no packet
 */
Packet::Packet() {
    this->header = this->packet; // header points to the top of the packet
    this->data = this->packet + PACKET_HEADER_SIZE; // data is offset PACKET_HEADER_SIZE bytes
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

    // Parse packet data and output individual fields
    // temperature data
    int temp = 0;
    temp |= this->data[1];
    temp <<= 8;
    temp |= this->data[0];
    Serial.print("Temperature: ");
    Serial.println(temp);
}
#endif /* DEBUG */

/**
 *
 */
void Packet::setHeader(uint8_t* header) {
    for (int i = 0; i < PACKET_HEADER_SIZE; i++)
        this->header[i] = header[i];
}

/**
 * Write to a specific byte in the packet data
 */
void Packet::setData(uint8_t byt, int index) {
    this->data[index] = byt;
}

/**
 * Copies the data
 */
void Packet::setData(uint8_t* data) {
    for (int i = 0; i < PACKET_DATA_SIZE; i++)
        this->data[i] = data[i];
}

/**
 * Encode sensor data into the packet
 */
void Packet::setField(PacketField field, void* data) {
    int temp;
    switch (field) {
        case Temperature:
            temp = *((int*)data);
            this->data[0] = temp & 0xff;
            this->data[1] = (temp >> 8) & 0xff;
            break;
#ifdef DEBUG
        default:
            Serial.println("PACKET: invalid packet field");
#endif /* DEBUG */
    }
}

/**
 * Returns a reference to the underlying array that is the packet
 */
uint8_t* Packet::toArray() {
    return this->packet;
}
