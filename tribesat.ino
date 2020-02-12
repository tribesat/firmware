/* TSLPB F2
 *  Flight Software: Serial Communication to ThinSat
 *  Reading the analog sensors
 *  Reading the digital Sensors
 *  Send them by serial port using microJSON format
 *
 *  -----------------------------------------------
 *  Digital Temperature DTx (LM75A)  A2 A1 A0
 *  I2C device found at address | 0x48  !   000 DT4
 *  I2C device found at address | 0x49  !   001 DT5
 *  I2C device found at address | 0x4A  !   010 DT1
 *  I2C device found at address | 0x4B  !   011 DT6 (Not used)
 *  I2C device found at address | 0x4C  !   100 DT2
 *  I2C device found at address | 0x4D  !   101 DT3
 *
 *  See microJSON_O1.xlsx file
 *
 */

#include <Wire.h>

// Library to use specific port with ThinSat Serial Comm
#include <SoftwareSerial.h>

#define SERIAL_BUSY 4        // Serial Busy line
#define LED_MONITOR 13       // LED monitor

// Mux = multiplexer shield. Used to turn an input into many outputs

#define mux A7        // ADC reading the MUX_Output
#define muxA 7        // Mux select A
#define muxB 8        // Mux select B
#define muxC 9        // Mux select C
//define magnetometry address
#define    MPU9250_ADDRESS            0x69    
#define    MAG_ADDRESS                0x0C


int sensors = 0;          // Value of the sensors coming fron the Mux
String data_string = "";  // for plotting
int mux_delay = 10;       // delay between samples on Mux
int i = 0;                // used as an auxiliary variable
int led = 0;              // if led = 1, led is on. if led = 0, led is off

byte payload_size = 38;   // Payload data is 35 bytes size (TODO: why 38?)
byte payload_packet[38];  // Payload packet 50 50 50 Payload(35 bytes)
int comm_status = 0;      // comm status = 1 means busy

byte RxByte=0;            // Incoming byte from Serial port
int read_delay = 10;      // wait duration needed between reads
int ACK = 0;              // ACK response from thinsat
int tries = 0;            // Tries to resend a package to the ThinsSat
int wait = 0;             // counter that will break out of serial monitor
                          // ACK loop if 50 tries is exceeded

uint32_t particle_count = 0;  // running count of charged particles the BG51
                              // sensor has registered in its lifetime

static const int RX_pin = 3, TX_pin = 5;  // ThinSat serial comm lines
SoftwareSerial thinsat_serial(RX_pin, TX_pin);    // Comm port for ThinSat

// intialize LED_MONITOR, SERIAL_BUSY, muxA, muxB, muxC
// start arduino's built in serial monitor and a SoftwareSerial thinsat_serial
// populate payload_packet array with starter data
// blink the LED to indicate the program has started
void setup() {
  Serial.begin(38400);
  pinMode(SERIAL_BUSY, INPUT);       // Serial Busy Line
  pinMode(LED_MONITOR, OUTPUT);      // Pin 13 output
  pinMode(muxA, OUTPUT);
  pinMode(muxB, OUTPUT);
  pinMode(muxC, OUTPUT);
  
  //I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);  // Set by pass mode for the magnetometers
  //I2CwriteByte(MAG_ADDRESS,0x0A,0x16);  // Request continuous magnetometer measurements in 16 bits

  payload_packet[0]= 0x50;          // Preamble
  payload_packet[1]= 0x50;          // Preamble
  payload_packet[2]= 0x50;          // Preamble

  payload_packet[3]= 0x01;          // NSL Reserved
  payload_packet[4]= 0x01;          // Seq. Count

  payload_packet[5]= 96;            // 1: ID_Lux
  payload_packet[8]= 97;            // 2: ID_IR
  payload_packet[11]= 98;           // 3: ID_IT
  payload_packet[14]= 99;           // 4: ID_ET
  payload_packet[17]= 100;          // 5: ID_Icc
  payload_packet[20]= 101;          // 6: ID_Vcc
  payload_packet[23]= 102;          // 7: ID_DT1
  payload_packet[26]= 103;          // 8: ID_DT2
  payload_packet[29]= 104;          // 9: ID_DT3
  payload_packet[32]= 105;          // 10: ID_DT4
  payload_packet[35]= 106;          // 11: ID_DT5

  payload_packet[38]= 13;           // End of frame (byte 39)

  thinsat_serial.begin(38400);      // begin Serial Comm for ThinSat
                                    // Baud rate is 38400

  Serial.flush();                   // clear the Serial buffer

  blinking();      // blinking LED indicates the start of the program
}

void loop() {
  comm_status = digitalRead(SERIAL_BUSY);  // if SERIAL_BUSY is true (1), wait.
  while (comm_status == 1) {
    comm_status = digitalRead(SERIAL_BUSY);
    // while SERIAL_BUSY is true, make sure LED is turned on
    digitalWrite(LED_MONITOR, HIGH);
  }
  // turn off LED once not busy
  digitalWrite(LED_MONITOR, LOW);
  
  // read I2C (magnetometry data)
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  
  /*int16_t mx=(Mag[1]<<8 | Mag[0]);
  int16_t my=(Mag[3]<<8 | Mag[2]);
  int16_t mz=(Mag[5]<<8 | Mag[4]);
  float conv_m=0.1465;   // convertion factor for +-4,800uT range 0.6uT???, 16bits>0.1465, 14bits>0.2929
  float mx_r=(mx)*conv_m;
  float my_r=(my)*conv_m;
  float mz_r=(mz)*conv_m;*/
  
  // read in solar sensor, infrared sensor, internal temperature,
  // external temperature, current monitor, and voltage monitor
  readMux();

  // read in digital temperature 1
  Read_DT1();

  // read in digital temperature 2
  Read_DT2();
 

  // read in digital temperature 3
  Read_DT3();

  // read in digital temperature 4
  Read_DT4();

  // read in the current count of particles detected
  // by the BG51 radiation sensor
  Read_BG51();

  ClearSerial();

  transmit_packet();         // transmit data


  //digitalWrite(LED_MONITOR, LOW);
  //delay(500);     //Delay for waiting the ACK or NAK
  while (thinsat_serial.available() == 0){
    wait=wait+1;
    delay(10);
    //Serial.println(wait);
    if (wait == 50){
      //Serial.println("BREAK");
      break;
      }   // To avoid infinite loop
  }
  //Serial.println(wait);
  wait=0;

  ReadACK();

  int comp=ACK;
  tries=0;
  while ( tries < 3 && comp == 0){
    transmit_packet();         //Transmit packet
    tries = tries +1;
    //Serial.print("Bad ACK:"); Serial.println(tries);
  }

  delay(500);

}

//**************************I2CReadByte()
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


//********************blinking
void blinking(){
  for (byte i=0; i<3; i++){
    digitalWrite(LED_MONITOR, HIGH);
    delay (100);
    digitalWrite(LED_MONITOR, LOW);
    delay (100);
  }
}



// reading in values from the multiplexer shield
void readMux(){
  digitalWrite(muxA, LOW);
  digitalWrite(muxB, LOW);
  digitalWrite(muxC, LOW);
  delay(mux_delay);
  sensors = analogRead(mux);

  // reading in solar sensor
  payload_packet[6]= highByte(sensors);  // convert from 2 bytes to 1 high
  payload_packet[7]= lowByte(sensors);   // convert from 2 bytes to 1 low


  //Mux_In_2   001 (IR)
  digitalWrite(muxA, HIGH);
  digitalWrite(muxB, LOW);
  digitalWrite(muxC, LOW);
  delay(mux_delay);
  sensors = analogRead(mux);

  // reading in infrared sensor data
  payload_packet[9]= highByte(sensors);  // convert from 2 bytes to 1 High
  payload_packet[10]= lowByte(sensors);  // convert from 2 bytes to 1 low


  //Mux_In_3   010 (Temp Int)
  digitalWrite(muxA, LOW);
  digitalWrite(muxB, HIGH);
  digitalWrite(muxC, LOW);
  delay(mux_delay);
  sensors = analogRead(mux);

  // reading in internal temperature sensor data
  payload_packet[12]= highByte(sensors);  // convert from 2 bytes to 1 High
  payload_packet[13]= lowByte(sensors);   // convert from 2 bytes to 1 low


  //Mux_In_4   011 (Temp Ext)
  digitalWrite(muxA, HIGH);
  digitalWrite(muxB, HIGH);
  digitalWrite(muxC, LOW);
  delay(mux_delay);
  sensors = analogRead(mux);

  // reading in external temperature sensor data
  payload_packet[15]= highByte(sensors);  // convert from 2 bytes to 1 High
  payload_packet[16]= lowByte(sensors);   // convert from 2 bytes to 1 low


  //Mux_In_5   100 (Current)
  digitalWrite(muxA, LOW);
  digitalWrite(muxB, LOW);
  digitalWrite(muxC, HIGH);
  delay(mux_delay);
  sensors = analogRead(mux);

  // reading in current montor data
  payload_packet[18]= highByte(sensors);  // convert from 2 bytes to 1 High
  payload_packet[19]= lowByte(sensors);   // convert from 2 bytes to 1 low


  //Mux_In_6   101 (Vcc)
  digitalWrite(muxA, HIGH);
  digitalWrite(muxB, LOW);
  digitalWrite(muxC, HIGH);
  delay(mux_delay);
  sensors = analogRead(mux);

  // reading in voltage monitor data
  payload_packet[21]= highByte(sensors);  // convert from 2 bytes to 1 High
  payload_packet[22]= lowByte(sensors);   // convert from 2 bytes to 1 low


  //Mux_In_7   110 (GND)
  digitalWrite(muxA, LOW);
  digitalWrite(muxB, HIGH);
  digitalWrite(muxC, HIGH);
  delay(mux_delay);
  sensors = analogRead(mux);
  // TODO: why are we reading in the above information if we don't do
  // anything with it

  //Mux_In_8   111 (PPS)
  digitalWrite(muxA, HIGH);
  digitalWrite(muxB, HIGH);
  digitalWrite(muxC, HIGH);
  delay(mux_delay);
  sensors = analogRead(mux);
  // TODO: same as above. what are we doing with this analog read
}

//********************transmit_packet
void transmit_packet(){
  thinsat_serial.write(payload_packet, payload_size);   // transmit payload

}


// reading in digital temperature 1 (bytes 24 and 25 in payload packet)
void Read_DT1(){
  int address=0x4A;
  Wire.requestFrom(address, 2);
  while(Wire.available()){
    // msb: most significant byte
    // lsb: least significant byte
    int8_t msb = Wire.read();
    int8_t lsb = Wire.read();
    // strip one bit of the lsb
    lsb = (lsb & 0x80 ) >> 7; // now lsb = 0 or 1

    // digital temperature 1
    payload_packet[24]= msb;
    payload_packet[25]= lsb;
  }

}

// reading in digital temperature 2 (bytes 27 and 28 in payload packet)
void Read_DT2(){
  int address=0x4C;
  Wire.requestFrom(address, 2);
  while(Wire.available()){
    // msb: most significant byte
    // lsb: least significant byte
    int8_t msb = Wire.read();
    int8_t lsb = Wire.read();
    lsb = (lsb & 0x80 ) >> 7; // now lsb = 0 or 1
    payload_packet[27]= msb;
    payload_packet[28]= lsb;
  }
}

// reading in digital temperature 3 (bytes 30 and 31 in payload packet)
void Read_DT3(){
  int address=0x4D;
  Wire.requestFrom(address, 2);
  while(Wire.available()){
    // msb: most significant byte
    // lsb: least significant byte
    int8_t msb = Wire.read();
    int8_t lsb = Wire.read();
    lsb = (lsb & 0x80 ) >> 7; // now lsb = 0 or 1
    payload_packet[30]= msb;
    payload_packet[31]= lsb;
  }
}

// reading in digital temperature 4 (bytes 33 and 34 in payload packet)
void Read_DT4(){
  int address=0x48;
  Wire.requestFrom(address, 2);
  while(Wire.available()){
    // msb: most significant byte
    // lsb: least significant byte
    int8_t msb = Wire.read();
    int8_t lsb = Wire.read();
    lsb = (lsb & 0x80 ) >> 7; // now lsb = 0 or 1
    payload_packet[33]= msb;
    payload_packet[34]= lsb;
  }
}


// reading in the count of charged particles (bytes 36 and 37 in payload packet)
void Read_BG51(){
  // TODO: confirm that it's ok for us to use this analog input port
  int address = A0;

  int voltage = analogRead(address);

  // if voltage is greater than at least 1 volt, we read a charged particle
  // TODO: this threshold value should be better validated --Liz
  // Converted voltage > 1 to voltage > 310 (of 1023 max) --Josh/Liz
  // Voltage is mapped from 0 to 3.3 V to 0 to 1023 int
  if(voltage > 310) {
    particle_count++;
  }

  payload_packet[36] = highByte(particle_count);
  payload_packet[37] = lowByte(particle_count);
}

//********************ReadACK
// TODO: ask about what this method is doing with its iterative reads
// of the health byte
void ReadACK(){
  ACK=0;
  // check to see if serial is available
  if (thinsat_serial.available() > 0){
    // read in health byte
    RxByte = thinsat_serial.read();
    if(RxByte == 0xAA){
      //Serial.print(RxByte);
      //Serial.print(",");
      delay(read_delay);
      RxByte = thinsat_serial.read();
      if(RxByte == 0x05){
        ACK = 0;
        //Serial.print(RxByte);
        //Serial.print(",");
        delay(read_delay);
        RxByte = thinsat_serial.read();
        if(RxByte == 0x00){
          //Serial.print(RxByte);
          ACK = 1;
        }
      }
    }
    /*
    RxByte = Serial.read();
    Serial.print(RxByte);
    Serial.print(",");
    RxByte = Serial.read();
    Serial.println(RxByte);
    */
   }
   //Serial.println(". DONE ACK");
}



//********************ClearSerial
void ClearSerial() {
 while(thinsat_serial.available()){  //is there anything to read?
  char getData = thinsat_serial.read();  //if yes, read it
  //Serial.print(getData);
  //Serial.print(",");
 }
 //Serial.println("Done");
}
