#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

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

sensor_msgs::Imu imu_msg;


ros::Publisher imu_pub("imu/data", &imu_msg);


void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  gyro.getSensor(&sensor);
  mag.getSensor(&sensor);
  bmp.getSensor(&sensor);
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

  nh.advertise( imu_pub );

}

void loop(void)
{
  imu_msg.header.frame_id = "imu";
  imu_msg.header.stamp = nh.now();
  //imu_msg.child_frame_id = "base_link";
  
  /* Get a new sensor event */
  sensors_event_t event;
  
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  /*
  float a_x = event.acceleration.x;
  float a_y = event.acceleration.y;
  float a_z = event.acceleration.z;
  */

  imu_msg.linear_acceleration.x = event.acceleration.x;
  imu_msg.linear_acceleration.y = event.acceleration.y;
  imu_msg.linear_acceleration.z = event.acceleration.z;
  

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  
  float g_x = event.gyro.x;
  float g_y = event.gyro.y;
  float g_z = event.gyro.z;

  imu_msg.angular_velocity.x = g_x;
  imu_msg.angular_velocity.y = g_y;
  imu_msg.angular_velocity.z = g_z;

  

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

  imu_pub.publish( &imu_msg );


  nh.spinOnce();
  delay(100);
}
