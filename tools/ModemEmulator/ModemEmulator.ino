/**
 * Tribesat Modem emulator
 */
#include <SoftwareSerial.h>

uint8_t ACK[] = { 0xAA, 0x05, 0x00 };
uint8_t NAK[] = { 0xAA, 0x05, 0xFF };

SoftwareSerial HardwareSerial(10, 11); // RX, TX

void printPacket(const uint8_t* packet) {
  Serial.print("[");
  for (int i = 0; i < 38; i++) {
    Serial.print(packet[i], HEX); Serial.print(',');
  }
  Serial.println("]");
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(57600);

  // wait for serial port to connect. Needed for Leonardo only
  while (!Serial);

  // set the data rate for the SoftwareSerial port
  HardwareSerial.begin(38400);

}

void loop() {
  // wait until an entire packet is available over serial
  while (HardwareSerial.available() < 38)
    delay(100);

  Serial.println("Reading packet");

  uint8_t* packet = new uint8_t[38];

  for (int i = 0; i < 38; i++)
      packet[i] = HardwareSerial.read();

  printPacket(packet);

  if (packet[0] == 0x50 && packet[0] == 0x50 && packet[0] == 0x50) {
    // ACK
    HardwareSerial.write(ACK, 3);
    Serial.println("Sending ACK");
  } else {
    // NAK
    HardwareSerial.write(NAK, 3);
    Serial.println("Sending NAK");
  }

  delete packet;
}
