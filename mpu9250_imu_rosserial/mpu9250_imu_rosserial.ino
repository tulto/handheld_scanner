#include <ESP8266WiFi.h>
#include "MPU9250.h"
//#include "eeprom_utils.h"
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>

MPU9250 mpu;

ros::NodeHandle nh;

geometry_msgs::Vector3 accel_msg;
geometry_msgs::Vector3 gyro_msg;
geometry_msgs::Vector3 orient_msg;
std_msgs::Header head_msg;

ros::Publisher ac("/imu/accel", &accel_msg);
ros::Publisher gy("/imu/gyro", &gyro_msg);
ros::Publisher ori("/imu/orient", &orient_msg);
ros::Publisher he("/imu/head", &head_msg);

int start;


void setup() {
  
  //Wire.begin();
  delay(500);
  mpu.setup(0x68);
  //loadCalibration();
  
  //mpu.verbose(true);
  //mpu.calibrateAccelGyro();
  //mpu.verbose(false);

  nh.initNode();
  
  nh.advertise(ac);
  nh.advertise(gy);
  nh.advertise(ori);
  nh.advertise(he);
  
  start = millis();

}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 4) {
            send_imu_data();
            prev_ms = millis();
        }
    }
    nh.spinOnce();
}

void send_imu_data() {
  //mpu.setGyroBias( mpu.getGyroBiasX(), mpu.getGyroBiasY(), mpu.getGyroBiasZ());
  //mpu.setAccBias( mpu.getAccBiasX(), mpu.getAccBiasY(), mpu.getAccBiasZ());

  accel_msg.x = mpu.getAccX()*9.81;
  accel_msg.y = mpu.getAccY()*9.81;
  accel_msg.z = mpu.getAccZ()*9.81;
  
  gyro_msg.x = mpu.getGyroX()/57.296;
  gyro_msg.y = mpu.getGyroY()/57.296;
  gyro_msg.z = mpu.getGyroZ()/57.296;
  
  orient_msg.x = mpu.getYaw()/57.296;
  orient_msg.y = mpu.getPitch()/57.296;
  orient_msg.z = mpu.getRoll()/57.296; 
  
  // sending header to get timing right
  head_msg.stamp = nh.now();
  head_msg.frame_id = "imu_link";
  //if (millis()> start + 5000){
  ac.publish(&accel_msg);
  gy.publish(&gyro_msg);
  ori.publish(&orient_msg);
  he.publish( &head_msg );   
  //}  
}
