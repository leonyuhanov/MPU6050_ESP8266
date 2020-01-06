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
long accellData[2][3], gyroData[2][3];
float rangedAccellData[3], rangedGyroData[3];
int accellFlutter[3] = {0, 0, 0};
int gyroFlutter[3] = {0, 0, 0};
int accellPeakValues[3] = {32768, 32768, 32768};
int gyroPeakValues[3] = {32768, 32768, 32768};
float accellFilters[3] = {0,0,0};
float gyroFilters[3] = {0,0,0};
int ticker=1;
int calibrationRuns = 1000;
byte calibrationDelay = 5;

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
  
  Serial.printf("\r\nStart Calibration routine for %d cycles...\r\n", calibrationRuns);
  for(ticker=0; ticker<calibrationRuns; ticker++)
  {
    sampleIMU();
    accellData[0][0] += AcX;
    accellData[0][1] += AcY;
    accellData[0][2] += AcZ;
    gyroData[0][0] += GyX;
    gyroData[0][1] += GyY;
    gyroData[0][2] += GyZ;
    delay(calibrationDelay);
    if( ticker%(calibrationRuns/10)==(calibrationRuns/10)-1)
    {
      Serial.printf("%d\t", ticker);
    }
  }
  Serial.printf("\r\nCalibration complete");
  accellData[1][0] = accellData[0][0]/calibrationRuns;
  accellData[1][1] = accellData[0][1]/calibrationRuns;
  accellData[1][2] = accellData[0][2]/calibrationRuns;
  gyroData[1][0] = gyroData[0][0]/calibrationRuns;
  gyroData[1][1] = gyroData[0][1]/calibrationRuns;
  gyroData[1][2] = gyroData[0][2]/calibrationRuns;
  Serial.printf("\r\nAX_Flutt\tAY_Flutt\tAZ_Flutt\t|\tGX_Flutt\tGY_Flutt\tGZ_Flutt");
  Serial.printf("\r\n%d\t%d\t%d\t|\t%d\t%d\t%d", accellData[1][0], accellData[1][1], accellData[1][2], gyroData[1][0], gyroData[1][1], gyroData[1][2]);
  //Set Accel avg flutter rates
  accellFlutter[0] = accellData[1][0];
  accellFlutter[1] = accellData[1][1];
  accellFlutter[2] = accellData[1][2];
  //Set GYRO avg flutter rates
  gyroFlutter[0] = gyroData[1][0];
  gyroFlutter[1] = gyroData[1][1];
  gyroFlutter[2] = gyroData[1][2];
  //Set Accel peak range
  accellPeakValues[0]-=abs(accellFlutter[0]);
  accellPeakValues[1]-=abs(accellFlutter[1]);
  accellPeakValues[2]-=abs(accellFlutter[2]);
  //Set Gyro peak range
  gyroPeakValues[0]-=abs(gyroFlutter[0]);
  gyroPeakValues[1]-=abs(gyroFlutter[1]);
  gyroPeakValues[2]-=abs(gyroFlutter[2]);
  delay(1000);
  Serial.printf("\r\nStart calibration of filters for %d cycles...\r\n", calibrationRuns);
  for(ticker=0; ticker<calibrationRuns; ticker++)
  {
    sampleIMU();
    accellData[0][0] = AcX-accellFlutter[0];
    accellData[0][1] = AcY-accellFlutter[1];
    accellData[0][2] = AcZ-accellFlutter[2];
    gyroData[0][0] = GyX-gyroFlutter[0];
    gyroData[0][1] = GyY-gyroFlutter[1];
    gyroData[0][2] = GyZ-gyroFlutter[2];

    accellFilters[0] += (((float)accellData[0][0])/accellPeakValues[0])*100;
    accellFilters[1] += (((float)accellData[0][1])/accellPeakValues[1])*100;
    accellFilters[2] += (((float)accellData[0][2])/accellPeakValues[2])*100;
    gyroFilters[0] += (((float)gyroData[0][0])/gyroPeakValues[0])*100;
    gyroFilters[1] += (((float)gyroData[0][1])/gyroPeakValues[1])*100;
    gyroFilters[2] += (((float)gyroData[0][2])/gyroPeakValues[2])*100;
    delay(calibrationDelay);
    if( ticker%(calibrationRuns/10)==(calibrationRuns/10)-1)
    {
      Serial.printf("%d\t", ticker);
    }
  }
  Serial.printf("\r\nFilter Calibration complete.");
  //set filter ranges for each axis
  accellFilters[0] = accellFilters[0]/calibrationRuns;
  accellFilters[1] = accellFilters[1]/calibrationRuns;
  accellFilters[2] = accellFilters[2]/calibrationRuns;
  gyroFilters[0] = gyroFilters[0]/calibrationRuns;
  gyroFilters[1] = gyroFilters[1]/calibrationRuns;
  gyroFilters[2] = gyroFilters[2]/calibrationRuns;
  delay(1000);
}

void loop()
{
  sampleIMU();
  //Read in raw values using the flutter rate to clip down to 0
  accellData[0][0] = AcX-accellFlutter[0];
  accellData[0][1] = AcY-accellFlutter[1];
  accellData[0][2] = AcZ-accellFlutter[2];
  gyroData[0][0] = GyX-gyroFlutter[0];
  gyroData[0][1] = GyY-gyroFlutter[1];
  gyroData[0][2] = GyZ-gyroFlutter[2];
  
  //calculate ratio based on peak
  rangedAccellData[0] = (((float)accellData[0][0])/accellPeakValues[0])*100;
  rangedAccellData[1] = (((float)accellData[0][1])/accellPeakValues[1])*100;
  rangedAccellData[2] = (((float)accellData[0][2])/accellPeakValues[2])*100;
  rangedGyroData[0] = (((float)gyroData[0][0])/gyroPeakValues[0])*100;
  rangedGyroData[1] = (((float)gyroData[0][1])/gyroPeakValues[1])*100;
  rangedGyroData[2] = (((float)gyroData[0][2])/gyroPeakValues[2])*100;

  if( rangedAccellData[0]<accellFilters[0] && rangedAccellData[0]>-accellFilters[0] ){rangedAccellData[0]=0;}
  if( rangedAccellData[1]<accellFilters[1] && rangedAccellData[1]>-accellFilters[1] ){rangedAccellData[1]=0;}
  if( rangedAccellData[2]<accellFilters[2] && rangedAccellData[2]>-accellFilters[2] ){rangedAccellData[2]=0;}
  if( rangedGyroData[0]<gyroFilters[0] && rangedGyroData[0]>-gyroFilters[0] ){rangedGyroData[0]=0;}
  if( rangedGyroData[1]<gyroFilters[1] && rangedGyroData[1]>-gyroFilters[1] ){rangedGyroData[1]=0;}
  if( rangedGyroData[2]<gyroFilters[2] && rangedGyroData[2]>-gyroFilters[2] ){rangedGyroData[2]=0;}
  
  Serial.printf("\r\n%f\t%f\t%f\t%f\t%f\t%f", rangedAccellData[0], rangedAccellData[1], rangedAccellData[2], rangedGyroData[0], rangedGyroData[1], rangedGyroData[2]);

  delay(50);
}

void sampleIMU()
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
}
