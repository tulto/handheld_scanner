// IMPORTANT
// this is not a working example, this is just to show how to set the library
// if you need a working example please see the other example


#include "SensorFusion.h" //SF
#include <MPU9250_WE.h>
#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#define MPU9250_ADDR 0x68
SF fusion;

float gx, gy, gz, ax, ay, az;//, mx, my, mz;
float deltat;

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

ros::NodeHandle nh;

//geometry_msgs::Quaternion orient_msg;
geometry_msgs::Quaternion data_msg;
std_msgs::Header head_msg;

ros::Publisher ori("/imu/orient", &data_msg);
ros::Publisher ac("/imu/accel", &data_msg);
ros::Publisher he("/imu/head", &head_msg);


void setup() {

  nh.initNode();
  
  nh.advertise(ori);
  nh.advertise(ac);
  nh.advertise(he);
  
  // your IMU begin code goes here
  Wire.begin();
  myMPU9250.init();
  //myMPU9250.initMagnetometer();


  myMPU9250.autoOffsets();
  
  /*  This is a more accurate method for calibration. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  Use either autoOffset or setAccOffsets, not both.
   */
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

  /*  The gyroscope data is not zero, even if you don't move the MPU9250. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffset or setGyrOffsets, not both.
   */
  //myMPU9250.setGyrOffsets(45.0, 145.0, -105.0);

  /*  You can enable or disable the digital low pass filter (DLPF). If you disable the DLPF, you 
   *  need to select the bandwdith, which can be either 8800 or 3600 Hz. 8800 Hz has a shorter delay,
   *  but higher noise level. If DLPF is disabled, the output rate is 32 kHz.
   *  MPU9250_BW_WO_DLPF_3600 
   *  MPU9250_BW_WO_DLPF_8800
   */
  myMPU9250.enableGyrDLPF();
  //myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // bandwdith without DLPF
  
  /*  Digital Low Pass Filter for the gyroscope must be enabled to choose the level. 
   *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7 
   *  
   *  DLPF    Bandwidth [Hz]   Delay [ms]   Output Rate [kHz]
   *    0         250            0.97             8
   *    1         184            2.9              1
   *    2          92            3.9              1
   *    3          41            5.9              1
   *    4          20            9.9              1
   *    5          10           17.85             1
   *    6           5           33.48             1
   *    7        3600            0.17             8
   *    
   *    You achieve lowest noise using level 6  
   */
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);

  /*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
   *  Sample rate = Internal sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
   *  Divider is a number 0...255
   */
  myMPU9250.setSampleRateDivider(5);

  /*  MPU9250_GYRO_RANGE_250       250 degrees per second (default)
   *  MPU9250_GYRO_RANGE_500       500 degrees per second
   *  MPU9250_GYRO_RANGE_1000     1000 degrees per second
   *  MPU9250_GYRO_RANGE_2000     2000 degrees per second
   */
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_1000);

  /*  MPU9250_ACC_RANGE_2G      2 g   (default)
   *  MPU9250_ACC_RANGE_4G      4 g
   *  MPU9250_ACC_RANGE_8G      8 g   
   *  MPU9250_ACC_RANGE_16G    16 g
   */
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);

  /*  Enable/disable the digital low pass filter for the accelerometer 
   *  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
   */
  myMPU9250.enableAccDLPF(true);

  /*  Digital low pass filter (DLPF) for the accelerometer, if enabled 
   *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7 
   *   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
   *     0           460               1.94           1
   *     1           184               5.80           1
   *     2            92               7.80           1
   *     3            41              11.80           1
   *     4            20              19.80           1
   *     5            10              35.70           1
   *     6             5              66.96           1
   *     7           460               1.94           1
   */
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);

  /* You can enable or disable the axes for gyroscope and/or accelerometer measurements.
   * By default all axes are enabled. Parameters are:  
   * MPU9250_ENABLE_XYZ  //all axes are enabled (default)
   * MPU9250_ENABLE_XY0  // X, Y enabled, Z disabled
   * MPU9250_ENABLE_X0Z   
   * MPU9250_ENABLE_X00
   * MPU9250_ENABLE_0YZ
   * MPU9250_ENABLE_0Y0
   * MPU9250_ENABLE_00Z
   * MPU9250_ENABLE_000  // all axes disabled
   */
  //myMPU9250.enableAccAxes(MPU9250_ENABLE_XYZ);
  //myMPU9250.enableGyrAxes(MPU9250_ENABLE_XYZ);
  
  /*
   * AK8963_PWR_DOWN       
   * AK8963_CONT_MODE_8HZ         default
   * AK8963_CONT_MODE_100HZ
   * AK8963_FUSE_ROM_ACC_MODE 
   */
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);
}

void loop() {

  // now you should read the gyroscope, accelerometer (and magnetometer if you have it also)
  // NOTE: the gyroscope data have to be in radians
  // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD
  imu_data();
  
  data_msg.x = fusion.getQuat()[2];
  data_msg.y = -fusion.getQuat()[1];
  data_msg.z = fusion.getQuat()[3];
  data_msg.w = fusion.getQuat()[0];
  ori.publish(&data_msg);

  

  head_msg.stamp = nh.now();
  head_msg.frame_id = "imu_link";
  he.publish( &head_msg ); 
  delay(10);
  nh.spinOnce();
}


void imu_data(){
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  //xyzFloat magValue = myMPU9250.getMagValues();
  //float resultantG = myMPU9250.getResultantG(gValue);
  
  gx = gyr.x/57.296;
  gy = gyr.y/57.296;
  gz = gyr.z/57.296;
  ax = gValue.x*9.81;
  ay = gValue.y*9.81;
  az = gValue.z*9.81;

  data_msg.x = ax;
  data_msg.y = ay;
  data_msg.z = az;
  ac.publish(&data_msg);

  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  //choose only one of these two:
  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate
  
}
