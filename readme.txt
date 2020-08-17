Visualize data from IMU connected to arduino and export to csv
(for Begginers)

Hardware Requirements
 - Arduino MEGA
 - LSM9DS1 Accelerometer + Gyro + Magnetometer 9-DOF Breakout (https://learn.adafruit.com/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout/overview)

Software Requirements
 - ROS kinetic
 - Arduino-1.8.13
 - Arduino libraries
  -ros_lib
  -Adafruit LSM9DS1
  -Adafruit Unified Sensor

If you haven't introduced ros_serial yet, please refer here
https://qiita.com/moriitkys/items/0b83fc6f99947059a34a


# Check that Arduino is connected and write this ino file to Arduino
imu_check_lsm9ds1.ino

# Check that Arduino is connected, and do this command
roslaunch imu_check imu_check.launch


