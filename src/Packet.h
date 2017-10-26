/**
 *
 */
#define DEBUG // TODO: REMOVE ME

#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>


#define ACK_SIZE 3
#define PACKET_HEADER_SIZE 3
#define PACKET_DATA_SIZE 35
#define PACKET_SIZE (PACKET_HEADER_SIZE + PACKET_DATA_SIZE)

enum PacketField {
    Temperature,
};

/**
 * Packet data structure
 */
class Packet {
  public:
    Packet();
    ~Packet();
    void setField(PacketField field, void* data);
    uint8_t* toArray();
#ifdef DEBUG
    void print();
#endif /* DEBUG */
  private:
    uint8_t* header;
    uint8_t* data;
    uint8_t packet[PACKET_SIZE];
};
#endif /* PACKET_H */
