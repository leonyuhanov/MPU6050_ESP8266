# MPU6050_ESP8266
Basic use of the MPU6050 IMU with an ESP8266

# CalibrationDataCode (Pre integration/raw data/not very usefull)
 - Grabs raw data from the the MPU6050
 - Calibrates avarage flutter rates of each axis
 - Calibrates RATIOED flutter rates for each axis. Accelerometer ratios are (+/-)0 to 100 and Gyro are (+/-)0 to sqrt(maxSenseValue)
 - Calibrates filters for each access to filter out noise and make output smoother
 - Outputs Ax,Ay,Az,Gx,Gy,Gz to the plotter using above calibrations

# CalibrationDataCode_INTEGRATION.ino
 - Grabs raw data from the the MPU6050
 - Calibrates avarage flutter rates of each axis
 - Calibrates RATIOED flutter rates for each axis
 - Calibrates filters for each access to filter out noise and make output smoother
 - Calculates ANGLES traveled by integrating Gyro data
 - Keeps Track of the current angular position of each axis
 - Outputs Gx,Gy,Gz Tracked angular data to the plotter using above calibrations
 
# MPU6050_CLASS
 - All of the above compiled into an easy to use C++ CLass
 
 Create the object

```C++
   mpu6050imu imuDevice;
 ```

Configure your ESP8266 Pin settings

```C++
  imuDevice.i2cDataPIN = D1;
  imuDevice.i2cClockPIN = D2;
```

Configure what ranges you want to use for the Accelerometer and the gyro or leave as 0 to default. To see details of this check out the regster map on https://www.i2cdevlib.com/devices/mpu6050#registers . 

```C++
  imuDevice.currentGyroRange = 0;
  imuDevice.currentAccellRange = 0;
```

Init the device:

```C++
  imuDevice.init();
```

Calibrate the sensor (make sure its in stil position not moving, not being vibrated or bumped). Calibrartion works out YOUR sensors noise rate at YOUR sensonrs current boot up & temperature. If you want to see output mesages from calibration make sure you set up your Serial.begin

```C++
  imuDevice.calibrate();
```

OPTIONALY: Set the acceleration & gyro trackign ranges

```C++
  //X Axis index 0
  axisIndex=0; 
  Set min tracking output for Accelerometer to 0
  imuDevice.accellTrackranges[axisIndex][0]=0;
  Set max tracking output for Accelerometer to 100
  imuDevice.accellTrackranges[axisIndex][1]=100;
  
  Set min tracking output for Gyro to 0
  imuDevice.gyroTrackranges[axisIndex][0]=0;
  Set max tracking output for Gyro to 100
  imuDevice.gyroTrackranges[axisIndex][1]=100;
```

To read tracked angular data simply loop this:

```C++
while(true)
{
 imuDevice.integration(360, 100);
 Serial.printf("\r\n%f\t%f\t%f", imuDevice.rangedAngularData[0], imuDevice.rangedAngularData[1], imuDevice.rangedAngularData[2]);
 delay(20);
}
```
