#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt32.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

ros::NodeHandle  nh;//ros node

//geometry_msgs::Twist imu_msg;
geometry_msgs::Vector3 accel_msg;
geometry_msgs::Vector3 gyro_msg;
//std_msgs::UInt32 t_msg;
std_msgs::Float64 t_msg;

ros::Publisher accel_pub("acceldata", &accel_msg);
ros::Publisher gyro_pub("gyrodata", &gyro_msg);
ros::Publisher t_pub("time", &t_msg);

unsigned long now_time;

void displaySensorDetails(void)
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  /*
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  */

  gyro.getSensor(&sensor);
  /*
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  */
  
  mag.getSensor(&sensor);
  /*
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  */

  bmp.getSensor(&sensor);
  /*
  Serial.println(F("-------- PRESSURE/ALTITUDE ---------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  */
  
  delay(500);
}

void setup(void)
{
  
  Serial.begin(57600);
  Serial.println(F("Adafruit 10DOF Tester")); Serial.println("");
  
  
  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    //Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    //Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    //Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    //Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

  nh.initNode();
  nh.advertise( accel_pub );
  nh.advertise( gyro_pub );
  nh.advertise( t_pub );
}

void loop(void)
{
  now_time = millis();
  /* Get a new sensor event */
  sensors_event_t event;
  
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  
  float a_x = event.acceleration.x;
  float a_y = event.acceleration.y;
  float a_z = event.acceleration.z;
  
  accel_msg.x= a_x;
  accel_msg.y= a_y;
  accel_msg.z= a_z;
  

  /*
  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  */

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  /*
  Serial.print(F("MAG   "));
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  */

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  
  float g_x = event.gyro.x;
  float g_y = event.gyro.y;
  float g_z = event.gyro.z;
  
  gyro_msg.x= g_x;
  gyro_msg.y= g_y;
  gyro_msg.z= g_z;
  

  //imu_msg.x= g_x;
  //imu_msg.y= g_y;
  //imu_msg.z= g_z;
  
  /*
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");  
  */

  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    /*
    Serial.print(F("PRESS "));
    Serial.print(event.pressure);
    Serial.print(F(" hPa, "));
    */
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /*
    Serial.print(temperature);
    Serial.print(F(" C, "));
    */
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    /*
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature)); 
    Serial.println(F(" m"));
    */
    
  }

  accel_pub.publish( &accel_msg );
  gyro_pub.publish( &gyro_msg );

  int nowtime = now_time;
  t_msg.data = nowtime;
  t_pub.publish( &t_msg );
  
  //Serial.println(F(""));
  nh.spinOnce();
  delay(100);
}
