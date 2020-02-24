#ifndef mpu6050imu_h
#define mpu6050imu_h
#include "Arduino.h"

class mpu6050imu
{
  public:
    mpu6050imu();
    void init();
    void readIMU();
    void calibrate();
    void integration(); 
    float getSlope(float x1, float x2);
    
    const int deviceI2CAddress = 0x68;
    byte i2cDataPIN;
    byte i2cClockPIN;

    byte currentGyroRange = 0;  //Sets Gyroscope range
    byte currentAccellRange =0; //Sets Accelerometer range
    const float gyroRangeSettings[4][2] = {{0,131},{8,65.5},{16,32.8},{48,16.4}};
    const float accellRangeSettings[4][2] = {{0,16384},{8,8192},{16,4096},{48,2048}};
    const byte gyroRangeSetting = (byte)gyroRangeSettings[currentGyroRange][0];       //sets the Gyroscope range on the IMU6050 at Address 0x1B 
    const byte accellRangeSetting = (byte)accellRangeSettings[currentAccellRange][0]; //sets the Accelerometer range on the IMU6050 at Address 0x1C
    const float gyroRangeDPS = gyroRangeSettings[currentGyroRange][1];
    const float accellRangeDPS = accellRangeSettings[currentAccellRange][1]; 
    //Temp Vars for reading in the raw data from the IMU
    int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
    long accellData[2][3];
    long gyroData[2][3];
    float rangedDistanceData[3];  //stores the real time current TRACKED angular position offset accellTrackranges[axisIndex][0] to accellTrackranges[axisIndex][1] bound to those min/max values
    float rangedAngularData[3];   //stores the real time current TRACKED angular position offset gyroTrackranges[axisIndex][0] to gyroTrackranges[axisIndex][1] bound to those min/max values
    int accellFlutter[3] = {0, 0, 0};
    int gyroFlutter[3] = {0, 0, 0};
    float accellFilters[3] = {0,0,0};
    float gyroFilters[3] = {0,0,0};
    short int accellTrackranges[3][2] = {{0,100}, {0,100}, {0,100}};
	  short int gyroTrackranges[3][2] = {{0,360}, {0,360}, {0,360}};
	  //Calibration settings
    unsigned short int calibrationRuns = 100;
    unsigned short int calibrationDelay = 1;
    unsigned short int caibrationMultiplier=3;
    unsigned short int sampleRate = 10;
    
    
    private:
      unsigned short int ticker=1;
      unsigned short int aCnt=0;
      long sampleTimer=1;
      unsigned short int debugMesageDelay=1000;
      float gyroSlope = 0;
      float integralGyroData[3] = {0,0,0}, integralAccellData[3] = {0,0,0};
      float timePerSlice;
      float tempAccellData[3] = {0,0,0}, tempGyroData[3] = {0,0,0};
  
};

#endif
