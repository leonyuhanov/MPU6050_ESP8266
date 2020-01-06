/*
Intented to output 
-the Centre Point of the Accelerometers & Gyros
-Average still drift for each sensor
	INSTRUCTIONS FOR USE
	-Upload to the ESP8266 module
	-Set the MPU6050 flat and as still as possible
	-Reset the ESP8266 and watch the serial output
*/
#include<Wire.h>
#include <ESP8266WiFi.h>

const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t accellData[2][3], gyroData[2][3];
int ticker=1;
//I2C Data & Clock pin foniguration, choose as you like
const byte dataPIN = D3, clockPIN = D4;


void setup()
{
  Serial.begin(115200);
  Serial.printf("\r\n\r\n");

  //Disable WIFI for Power save 
  WiFi.mode(WIFI_OFF);
  //Init i2c and set up the MPU6050
  Wire.begin(dataPIN, clockPIN);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.printf("\r\nAcc-X\tAcc-Y\tAcc-Z\tGyr-X\tGyr-Y\tGyr-Z\tFLUT_AX\tFLUT_AY\tFLUT_AZ\tFLUT_GX\tFLUT_GY\tFLUT_GZ\t");
}

void loop()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,1);
  
  //Read X,Y & Z accelerometer data
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  //Temperature(not used for anything n this code)
  Tmp=Wire.read()<<8|Wire.read();
  //Read X,Y & Z Gyro data
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
  
  //Accumulate Accelerometer readings for Averraging the flutter variance of each axis
  accellData[0][0] += AcX;
  accellData[0][1] += AcY;
  accellData[0][2] += AcZ;
  //Calculate avg flutter
  accellData[1][0] = accellData[0][0]/ticker;
  accellData[1][1] = accellData[0][1]/ticker;
  accellData[1][2] = accellData[0][2]/ticker;
  //Accumulate Gyro readings for Averraging the flutter variance of each axis
  gyroData[0][0] += GyX;
  gyroData[0][1] += GyY;
  gyroData[0][2] += GyZ;
  //Calculate avg flutter
  gyroData[1][0] = gyroData[0][0]/ticker;
  gyroData[1][1] = gyroData[0][1]/ticker;
  gyroData[1][2] = gyroData[0][2]/ticker;
  ticker++;
  
  /*
  Print out the following table:
  Accel-X|Accel-Y|Accel-Z|Gyro-X|Gyro-Y|Gyro-Z|Accel-X_FLUTTER|Accel-Y_FLUTTER|Accel-Z_FLUTTER|Gyro-X_FLUTTER|Gyro-Y_FLUTTER|Gyro-Z_FLUTTER
  */
  Serial.printf("\r\n%d\t%d\t%d\t%d\t%d\t%d", AcX, AcY, AcZ, GyX, GyY, GyZ);
  Serial.printf("\t%d\t%d\t%d\t%d\t%d\t%d", accellData[1][0], accellData[1][1], accellData[1][2], gyroData[1][0], gyroData[1][1], gyroData[1][2]);
 
  delay(50);
}
