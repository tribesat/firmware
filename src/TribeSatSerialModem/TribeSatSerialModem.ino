/*
  Software serial multple serial test
 
 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.
 
 The circuit: 
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)
 
 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts, 
 so only the following can be used for RX: 
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
 
 Not all pins on the Leonardo support change interrupts, 
 so only the following can be used for RX: 
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
 
 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example
 
 This example code is in the public domain.
 
 */
#include <SoftwareSerial.h>

SoftwareSerial HardwareSerial(10, 11); // RX, TX

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  HardwareSerial.begin(38400);
  HardwareSerial.println("Hello, world?");
}

void printPacket(const uint8_t* packet) {
  for (int i = 0; i < 38; i++) {
    Serial.print(packet[i], HEX); Serial.print(',');
  }
  Serial.println();
}

void loop() // run over and over
{
  if (HardwareSerial.available()) {
    
    uint8_t* packet = new uint8_t[39];
    for (int i = 0; i < 38; i++) {
        packet[i] = HardwareSerial.read();
    }
    packet[38] = 0;
    if (packet[0] == 0x50 && packet[0] == 0x50 && packet[0] == 0x50) {
      // ACK
      uint8_t ack[3]; ack[0] = 0xAA; ack[1] = 0x05; ack[2] = 0x00;
      HardwareSerial.write(ack,3);
      
      printPacket(packet);
      
    } else {
      // NAK
      uint8_t nak[3]; nak[0] = 0xAA; nak[1] = 0x05; nak[2] = 0xFF;
      HardwareSerial.write(nak,3);
    }
    
    delete packet;
  }
}

