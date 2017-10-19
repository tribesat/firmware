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

void printPacket(const uint8_t* packet, const int len = 38) {
  for (int i = 0; i < len; i++) {
    Serial.print(packet[i], HEX); Serial.print(',');
  }
  Serial.println();
}

void loop() // run over and over
{
  if (Serial.available()) {
    char buf = Serial.read();
    Serial.write(buf);
    
    uint8_t* packet = new uint8_t[38];
    packet[0] = 0x50;
    packet[1] = 0x50;
    packet[2] = 0x50;
    packet[3] = buf;
    packet[4] = 0;
    
    HardwareSerial.write(packet,38);

    // wait for ACK or NAK
    int len = 0;
    while ((len = HardwareSerial.available()) <= 3) {
      delay(100);
    }
    uint8_t* ackornak = new uint8_t[len];
    for (int i = 0; i < len; i++) {
      ackornak[i] = HardwareSerial.read();
    }
    printPacket(ackornak,len);
    if (ackornak[2] == 0xFF) {
      Serial.write("!");
    } else {
      Serial.write(".");
    }
    
    delete packet;
  }
}

