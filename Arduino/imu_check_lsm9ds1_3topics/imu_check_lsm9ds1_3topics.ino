#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_LSM9DS1.h>

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

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // i2c sensor


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

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void setup(void)
{
  
  Serial.begin(57600);

  nh.initNode();
  nh.advertise( accel_pub );
  nh.advertise( gyro_pub );
  nh.advertise( t_pub );

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  setupSensor();
}

void loop(void)
{
  now_time = millis();
  
  lsm.read();  /* ask it to read in the data */ 

  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp);

  
  float a_x = accel.acceleration.x;
  float a_y = accel.acceleration.y;
  float a_z = accel.acceleration.z;
  
  accel_msg.x= a_x;
  accel_msg.y= a_y;
  accel_msg.z= a_z;


  float g_x = gyro.gyro.x;
  float g_y = gyro.gyro.y;
  float g_z = gyro.gyro.z;
  
  gyro_msg.x= g_x;
  gyro_msg.y= g_y;
  gyro_msg.z= g_z;

  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(accel.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(accel.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(mag.magnetic.x);   Serial.print(" uT");
  Serial.print("\tY: "); Serial.print(mag.magnetic.y);     Serial.print(" uT");
  Serial.print("\tZ: "); Serial.print(mag.magnetic.z);     Serial.println(" uT");

  Serial.print("Gyro X: "); Serial.print(gyro.gyro.x);   Serial.print(" rad/s");
  Serial.print("\tY: "); Serial.print(gyro.gyro.y);      Serial.print(" rad/s");
  Serial.print("\tZ: "); Serial.print(gyro.gyro.z);      Serial.println(" rad/s");

  accel_pub.publish( &accel_msg );
  gyro_pub.publish( &gyro_msg );

  int nowtime = now_time;
  t_msg.data = nowtime;
  t_pub.publish( &t_msg );
  
  //Serial.println(F(""));
  nh.spinOnce();
  delay(100);
}
