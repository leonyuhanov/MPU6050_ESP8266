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
float rangedAccellData[3], rangedGyroData[3], rangedAngularData[3], prevRangedGyroData[3], prevRangedAccellData[3];
int accellFlutter[3] = {0, 0, 0};
int gyroFlutter[3] = {0, 0, 0};
int accellPeakValues[3] = {32768, 32768, 32768};
int gyroPeakValues[3] = {32768, 32768, 32768};
float accellFilters[3] = {0,0,0};
float gyroFilters[3] = {0,0,0};
int ticker=1, aCnt=0;
int calibrationRuns = 500, calibrationDelay = 5, caibrationMultiplier=2, sampleRate = 10;
short int maxRotationMultiplier[3] = {22, 22, 22};
//I2C Data & Clock pin foniguration, choose as you like
const byte dataPIN = D1, clockPIN = D2;


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
  
  Serial.printf("\r\nStart Acc/Gyro Calibration routine for standing noise %d cycles...\r\n", calibrationRuns);
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
  for(aCnt=0; aCnt<3; aCnt++)
  {
    accellData[1][aCnt] = accellData[0][aCnt]/calibrationRuns;
    gyroData[1][aCnt] = gyroData[0][aCnt]/calibrationRuns;
  }
  Serial.printf("\r\nStanding Noise Calibration complete.");
  Serial.printf("\r\n\tAX_Flutt\tAY_Flutt\tAZ_Flutt\t|\tGX_Flutt\tGY_Flutt\tGZ_Flutt");
  Serial.printf("\r\n\t%d\t%d\t%d\t\t\t|\t\t%d\t%d\t%d", accellData[1][0], accellData[1][1], accellData[1][2], gyroData[1][0], gyroData[1][1], gyroData[1][2]);
  //Set Accel avg flutter rates
  //Set GYRO avg flutter rates
  for(aCnt=0; aCnt<3; aCnt++)
  {
    accellFlutter[aCnt] = accellData[1][aCnt];
    gyroFlutter[aCnt] = gyroData[1][aCnt];
  }
  //Set Accel peak range
  //Set Gyro peak range
  for(aCnt=0; aCnt<3; aCnt++)
  {
    accellPeakValues[aCnt] -= abs(accellFlutter[aCnt]);
    gyroPeakValues[aCnt] -= abs(gyroFlutter[aCnt]);
    gyroPeakValues[aCnt] = sqrt(gyroPeakValues[aCnt]);
  }
  delay(1000);
  Serial.printf("\r\nStart calibration of input filters for %d cycles...\r\n", calibrationRuns);
  for(ticker=0; ticker<calibrationRuns; ticker++)
  {
    sampleIMU();
    accellData[0][0] = AcX-accellFlutter[0];
    accellData[0][1] = AcY-accellFlutter[1];
    accellData[0][2] = AcZ-accellFlutter[2];
    gyroData[0][0] = GyX-gyroFlutter[0];
    gyroData[0][1] = GyY-gyroFlutter[1];
    gyroData[0][2] = GyZ-gyroFlutter[2];

    for(aCnt=0; aCnt<3; aCnt++)
    {
      accellFilters[aCnt] += ((float)accellData[0][aCnt])/accellPeakValues[aCnt];
      if(gyroData[0][aCnt]>0)
      {
        gyroFilters[aCnt] +=  sqrt( gyroData[0][aCnt] ) / gyroPeakValues[aCnt] ;
      }
      else
      {
        gyroFilters[aCnt] +=  sqrt( abs(gyroData[0][aCnt]) ) / gyroPeakValues[aCnt] ;
      }
    }
    delay(calibrationDelay);
    if( ticker%(calibrationRuns/10)==(calibrationRuns/10)-1)
    {
      Serial.printf("%d\t", ticker);
    }
  }
  //set filter ranges for each axis
  for(ticker=0; ticker<3; ticker++)
  {
    accellFilters[ticker] = (accellFilters[ticker]/calibrationRuns)*caibrationMultiplier;
    gyroFilters[ticker] = (gyroFilters[ticker]/calibrationRuns)*caibrationMultiplier;
  }  
  Serial.printf("\r\nFilter Calibration complete....");
  Serial.printf("\r\nAx\t%f\tAy\t%f\tAz\t%f\tGx\t%f\tGy\t%f\tGz\t%f", accellFilters[0],accellFilters[1],accellFilters[2],gyroFilters[0],gyroFilters[1],gyroFilters[2]);
  Serial.printf("\r\nIMU CALIBRATION COMPLETE\r\n");
  delay(2000);
}

void loop()
{
  calcIMUData();
  trackAngle();
  Serial.printf("\r\n%f\t%f\t%d", rangedGyroData[0]*100, rangedAngularData[0], ((int)rangedAngularData[0])%360 );
  delay(20);
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

void calcIMUData()
{
  //Reset Accell/Gyro Counters
  for(aCnt=0; aCnt<3; aCnt++)
  {
    //Store previous readings to analyise gradient slope direction
    prevRangedAccellData[aCnt] = rangedAccellData[aCnt];
    prevRangedGyroData[aCnt] = rangedGyroData[aCnt];
    //clear out values for this read
    accellData[0][aCnt] = 0;
    gyroData[0][aCnt]=0;
  }
  //Take sampleRate number of samples for averaging  
  for(aCnt=0; aCnt<sampleRate; aCnt++)
  {
    sampleIMU();
    //Read in raw values
    accellData[0][0] += AcX-accellFlutter[0];
    accellData[0][1] += AcY-accellFlutter[1];
    accellData[0][2] += AcZ-accellFlutter[2];
    gyroData[0][0] += GyX-gyroFlutter[0];
    gyroData[0][1] += GyY-gyroFlutter[1];
    gyroData[0][2] += GyZ-gyroFlutter[2];
  }
  for(aCnt=0; aCnt<3; aCnt++)
  {
    accellData[0][aCnt] = accellData[0][aCnt]/sampleRate;
    gyroData[0][aCnt] = gyroData[0][aCnt]/sampleRate;
  }
  
  //calculate ratio based on peak
  for(aCnt=0; aCnt<3; aCnt++)
  {
    rangedAccellData[aCnt] = ((float)accellData[0][aCnt]) / accellPeakValues[aCnt];
    if(gyroData[0][aCnt]>0)
    {
      rangedGyroData[aCnt] = sqrt( gyroData[0][aCnt] ) / gyroPeakValues[aCnt];
    }
    else
    {
      rangedGyroData[aCnt] = -( sqrt( abs(gyroData[0][aCnt]) ) / gyroPeakValues[aCnt] );
    }
    
    if( rangedAccellData[aCnt]<accellFilters[aCnt] && rangedAccellData[aCnt]>-accellFilters[aCnt] ){rangedAccellData[aCnt]=0;}
    if( rangedGyroData[aCnt]<gyroFilters[aCnt] && rangedGyroData[aCnt]>-gyroFilters[aCnt] ){rangedGyroData[aCnt]=0;}
  }
}

float getSlope(float x1, float x2)
{
  float slope=0;
  
  if(x1!=x2)
  {
    slope = 1 / (x2-x1);
  }
  return slope;
}

void trackAngle()
{
  float currentSlope = 0;
  for(ticker=0; ticker<3; ticker++)
  {
    currentSlope = getSlope(prevRangedGyroData[ticker], rangedGyroData[ticker]);
    if(currentSlope>0)
    {
        rangedAngularData[ticker] += (maxRotationMultiplier[ticker]*rangedGyroData[ticker]);
    }
    else if(currentSlope<0)
    {
        rangedAngularData[ticker] -= maxRotationMultiplier[ticker] * abs(rangedGyroData[ticker]);
    }
  }
}
