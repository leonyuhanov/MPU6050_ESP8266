#include "mpu6050imu.h"
#include<Wire.h>


mpu6050imu::mpu6050imu()
{    
  
}

void mpu6050imu::init()
{
  //Init i2c and set up the MPU6050
  Wire.begin(i2cDataPIN, i2cClockPIN);
  //Reset MPU6050
  Wire.beginTransmission(deviceI2CAddress);
  Wire.write(0x6B); 
  Wire.write(0);
  Wire.endTransmission(true);   
  //Set Gyro Range
  Wire.beginTransmission(deviceI2CAddress);
  Wire.write(0x1B); 
  Wire.write(gyroRangeSetting);  
  Wire.endTransmission(true);
  //Set Accell Range
  Wire.beginTransmission(deviceI2CAddress);
  Wire.write(0x1C); 
  Wire.write(accellRangeSetting);  
  Wire.endTransmission(true);
}

void mpu6050imu::calibrate()
{
  Serial.printf("\r\nStart Acc/Gyro Calibration routine for standing noise %d cycles...\r\n", calibrationRuns);
  for(ticker=0; ticker<calibrationRuns; ticker++)
  {
    readIMU();
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
  delay(debugMesageDelay);
  Serial.printf("\r\nStart calibration of input filters for %d cycles...\r\n", calibrationRuns);
  //Set Accel avg flutter rates
  //Set GYRO avg flutter rates
  for(aCnt=0; aCnt<3; aCnt++)
  {
    accellFlutter[aCnt] = accellData[1][aCnt];
    gyroFlutter[aCnt] = gyroData[1][aCnt];
  }
  for(ticker=0; ticker<calibrationRuns; ticker++)
  {
    readIMU();
    accellData[0][0] = AcX-accellFlutter[0];
    accellData[0][1] = AcY-accellFlutter[1];
    accellData[0][2] = AcZ-accellFlutter[2];
    gyroData[0][0] = GyX-gyroFlutter[0];
    gyroData[0][1] = GyY-gyroFlutter[1];
    gyroData[0][2] = GyZ-gyroFlutter[2];

    for(aCnt=0; aCnt<3; aCnt++)
    {
      accellFilters[aCnt] += ((float)accellData[0][aCnt]) / accellRangeSettings[currentAccellRange][1];
      gyroFilters[aCnt] +=  ((float)gyroData[0][aCnt])  / gyroRangeSettings[currentGyroRange][1];
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
  delay(debugMesageDelay);
}

void mpu6050imu::readIMU()
{
  Wire.beginTransmission(deviceI2CAddress);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(deviceI2CAddress,14,1);
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

void mpu6050imu::integration()
{
  //float timePerSlice = ((float)(millis()-sampleTimer))/1000;
  float timePerSlice = ((float)(millis()-sampleTimer));
  
  //zero out internal counters
  for(aCnt=0; aCnt<3; aCnt++)
  {
    integralGyroData[aCnt] = 0;
    integralAccellData[aCnt] = 0;
    tempAccellData[aCnt] = 0;
    tempGyroData[aCnt] = 0;
  }
  
  readIMU();
  accellData[0][0] = AcX-accellFlutter[0];
  accellData[0][1] = AcY-accellFlutter[1];
  accellData[0][2] = AcZ-accellFlutter[2];
  gyroData[0][0] = GyX-gyroFlutter[0];
  gyroData[0][1] = GyY-gyroFlutter[1];
  gyroData[0][2] = GyZ-gyroFlutter[2];

  //add integral
  for(aCnt=0; aCnt<3; aCnt++)
  {
    tempAccellData[aCnt] = ((float)accellData[0][aCnt])/accellRangeDPS;
    tempGyroData[aCnt] = ((float)gyroData[0][aCnt])/gyroRangeDPS; 
    
    if(tempAccellData[aCnt]>accellFilters[aCnt] || tempAccellData[aCnt]<-accellFilters[aCnt])
    {
      integralAccellData[aCnt] += tempAccellData[aCnt] / timePerSlice;
    }
    if(tempGyroData[aCnt]>gyroFilters[aCnt] || tempGyroData[aCnt]<-gyroFilters[aCnt])
    {
      //integralGyroData[aCnt] += tempGyroData[aCnt] * timePerSlice;
      integralGyroData[aCnt] += tempGyroData[aCnt] / timePerSlice;
    }
  }

  //Add area to angular tracker
  for(aCnt=0; aCnt<3; aCnt++)
  {
    rangedAngularData[aCnt] += integralGyroData[aCnt];
    rangedDistanceData[aCnt] += integralAccellData[aCnt];
    
    //limit filters for gyro
    if(rangedAngularData[aCnt]>gyroTrackranges[aCnt][1])
    {
      rangedAngularData[aCnt]=gyroTrackranges[aCnt][1];
    }
    else if(rangedAngularData[aCnt]<gyroTrackranges[aCnt][0])
    {
      rangedAngularData[aCnt]=gyroTrackranges[aCnt][0];
    }
    //limit filters for accell
    if(rangedDistanceData[aCnt]>accellTrackranges[aCnt][1])
    {
      rangedDistanceData[aCnt]=accellTrackranges[aCnt][1];
    }
    else if(rangedDistanceData[aCnt]<accellTrackranges[aCnt][0])
    {
      rangedDistanceData[aCnt]=accellTrackranges[aCnt][0];
    }
  }
  
  sampleTimer = millis();
}

float mpu6050imu::getSlope(float x1, float x2)
{
  float slope=0;
  
  if(x1!=x2)
  {
    slope = 1 / (x2-x1);
  }
  return slope;
}
