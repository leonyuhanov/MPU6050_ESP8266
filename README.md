# MPU6050_ESP8266
Basic use of the MPU6050 IMU with an ESP8266

# CalibrationDataCode
 - Grabs raw data from the the MPU6050
 - Calibrates avarage flutter rates of each axis
 - Calibrates RATIOED flutter rates for each axis. Accelerometer ratios are (+/-)0 to 100 and Gyro are (+/-)0 to sqrt(maxSenseValue)
 - Calibrates filters for each access to filter out noise and make output smoother
 - Outputs Ax,Ay,Az,Gx,Gy,Gz to the plotter using above calibrations
