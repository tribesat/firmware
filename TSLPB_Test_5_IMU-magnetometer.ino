/*
 * IMU Test
 * Temp, Acc_X, Acc_Y, Acc_Z, Gyr_X, Gyr_Y, Gyr_Z, Mag_X, Mag_Y, Mag_Z
 */

#include <Wire.h>

#define    MPU9250_ADDRESS            0x69    
#define    MAG_ADDRESS                0x0C

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);  // Set by pass mode for the magnetometers
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);  // Request continuous magnetometer measurements in 16 bits
  
}


void loop() {
=
  // :::  Magnetometer ::: 

  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=(Mag[1]<<8 | Mag[0]);
  int16_t my=(Mag[3]<<8 | Mag[2]);
  int16_t mz=(Mag[5]<<8 | Mag[4]);
  /*
  int16_t mx=-(Mag[3]<<8 | Mag[2]);
  int16_t my=-(Mag[1]<<8 | Mag[0]);
  int16_t mz=-(Mag[5]<<8 | Mag[4]);
  */
    // real magnetometer NO declination adjust value in uT
  
  //float conv_m=1;
  float conv_m=0.1465;   // convertion factor for +-4,800uT range 0.6uT???, 16bits>0.1465, 14bits>0.2929
  float mx_r=(mx)*conv_m;
  float my_r=(my)*conv_m;
  float mz_r=(mz)*conv_m;

  
  
  // Magnetometer
  /*
  Serial.print (mx+200,DEC); 
  Serial.print ("\t");
  Serial.print (my-70,DEC);
  Serial.print ("\t");
  Serial.print (mz-700,DEC);  
  Serial.print ("\t");
  */
  /*  
  Serial.print (mx,DEC); 
  Serial.print ("\t");
  Serial.print (my,DEC);
  Serial.print ("\t");
  Serial.print (mz,DEC);  
  Serial.print ("\t");
  */
   //Display the results (magnetic vector values are in micro-Tesla (uT))
  Serial.print (mx_r,1); 
  Serial.print ("\t");
  Serial.print (my_r,1);
  Serial.print ("\t");
  Serial.print (mz_r,1);  
  Serial.print ("\t");

  
  // End of line
  Serial.println("");
//  delay(100);    
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


//******************************I2CwriteByte()
//Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}







