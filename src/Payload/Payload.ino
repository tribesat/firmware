/**
 * Tribesat firmware entrypoint
 */
#include <SoftwareSerial.h>
#include <Wire.h>
#include "SparkFunTMP102.h"
#include "Packet.h"
#include "Comms.h"

#define DEBUG

SoftwareSerial ModemSerial(10, 11); // RX, TX

uint8_t ackOrNak[ACK_SIZE];
Packet *packet = new Packet();

// For Adafruit TMP102
const int TEMP_ALERT_PIN = A3;
TMP102 tempSensor(0x48); // Initialize sensor at I2C address 0x48

/**
 *
 */
int readTempSensor() {
  tempSensor.wakeup(); // turn sensor on
  int temperature = (int)tempSensor.readTempC(); // read temperature data
  tempSensor.sleep(); // put sensor in sleep mode
  return temperature;
}

/**
 *
 */
bool sendPacket(Packet* packet) {
  int retryCount = SEND_NUM_RETRIES;

#ifdef DEBUG
  Serial.print("sending packet: "); packet->print();
#endif /* DEBUG */

  ModemSerial.write(packet->toArray(), PACKET_SIZE);

  // read ACK or NAK - wait for 3 bytes to be available
  while (ModemSerial.available() < ACK_SIZE)
    delay(100);

  for (int i = 0; i < ACK_SIZE; i++)
    ackOrNak[i] = ModemSerial.read();

  while (!isACK(ackOrNak) && retryCount) {
    ModemSerial.write(packet->toArray(), PACKET_SIZE);

    // read ACK or NAK - wait for 3 bytes to be written
    while (ModemSerial.available() < ACK_SIZE)
      delay(100);

    for (int i = 0; i < ACK_SIZE; i++)
      ackOrNak[i] = ModemSerial.read();

    retryCount--;
  }

  bool success = isACK(ackOrNak);

#ifdef DEBUG
  if (!success)
    Serial.print("Failed to send packet after "); Serial.print(SEND_NUM_RETRIES); Serial.println(" attemps");
#endif /* DEBUG */

  return success;
}

/**
 * set up TMP102
 */
void initTempSensor() {
  pinMode(TEMP_ALERT_PIN, INPUT);
  tempSensor.begin();  // Join I2C bus

  tempSensor.setFault(0);  // Trigger alarm immediately

  // Initialize tempSensor settings
  // These settings are saved in the sensor, even if it loses power

  // set the number of consecutive faults before triggering alarm.
  // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
  tempSensor.setFault(0);  // Trigger alarm immediately

  // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
  tempSensor.setAlertPolarity(1); // Active HIGH

  // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
  tempSensor.setAlertMode(0); // Comparator Mode.

  // set the Conversion Rate (how quickly the sensor gets a new reading)
  // 0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  tempSensor.setConversionRate(2);

  // set Extended Mode.
  // 0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  tempSensor.setExtendedMode(0);

  // set T_HIGH, the upper limit to trigger the alert on
  tempSensor.setHighTempC(29.4); // set T_HIGH in C

  // set T_LOW, the lower limit to shut turn off the alert
  tempSensor.setLowTempC(26.67); // set T_LOW in C

#ifdef DEBUG
  Serial.println("TEMP: Temperature sensor setup");
#endif /* DEBUG */
}

/**
 *
 */
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(57600);

  // set the data rate for the SoftwareSerial port
  ModemSerial.begin(38400);

  // set up temperature sensor
  initTempSensor();
}

/**
 *
 */
void loop() {
  int temp = readTempSensor();
  packet->setField(Temperature, &temp);
  bool success = sendPacket(packet);

#ifdef DEBUG
  if (success) {
    Serial.println("Successfully sent packet");
  } else {
    Serial.println("Failed to send packet");
  }
#endif
}
