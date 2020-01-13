/*
Intented to output 
-the Centre Point of the Accelerometers & Gyros
-Average still drift for each sensor
	INSTRUCTIONS FOR USE
	-Upload to the ESP8266 module
	-Set the MPU6050 flat and as still as possible
	-Reset the ESP8266 and watch the serial output
*/
#include <ESP8266WiFi.h>
#include "mpu6050imu.h"

mpu6050imu imuDevice;

void setup()
{
  Serial.begin(115200);
  Serial.printf("\r\n\r\n");

  //Disable WIFI for Power save 
  WiFi.mode(WIFI_OFF);
  //Configure MPU6050
  imuDevice.i2cDataPIN = D1;
  imuDevice.i2cClockPIN = D2;
  imuDevice.currentGyroRange = 0;
  imuDevice.currentAccellRange = 0;
  imuDevice.init();
  //Initiate MPU6050 Calibration routine
  imuDevice.calibrate();
}

void loop()
{
  imuDevice.integration();
  Serial.printf("\r\n%f\t%f\t%f", imuDevice.rangedAngularData[0], imuDevice.rangedAngularData[1], imuDevice.rangedAngularData[2]);
  delay(20);
}
