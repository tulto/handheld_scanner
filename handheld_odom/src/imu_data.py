#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

# global variables for separated msgs for better Performance
msg_orient = Quaternion()
msg_accel = Vector3()
msg_gyro = Vector3()

# Parameters of the Covariance Matrices will be found or set
if not rospy.has_param('odometry_imu_node/orientation_covariance'):
    rospy.set_param('odometry_imu_node/orientation_covariance', [0.1, 0, 0,
                                                       0, 0.1, 0,
                                                       0, 0, 0.1])

if not rospy.has_param('odometry_imu_node/accel_covariance'):
    rospy.set_param('odometry_imu_node/accel_covariance', [0.1, 0, 0,
                                                      0, 0.1, 0,
                                                      0, 0, 0.1])

if not rospy.has_param('odometry_imu_node/gyro_covariance'):
    rospy.set_param('odometry_imu_node/gyro_covariance', [0.4, 0, 0,
                                                     0, 0.4, 0,
                                                     0, 0, 0.4])
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

# imu orientations data will be saved inside global variable
def imu_orient(msg):
    global msg_orient  # to get global data point
    msg_orient.x = msg.x
    msg_orient.y = msg.y
    msg_orient.z = msg.z
    msg_orient.w = msg.w


# imu acceleration data will be saved inside global variable
def imu_accel(msg):
    global msg_accel  # to get global data point
    msg_accel = msg


# imu gyro data will be saved inside global variable
def imu_gyro(msg):
    global msg_gyro  # to get global data point
    msg_gyro = msg


# imu header data will be used to stich all msgs together
def imu_head(msg):
    msg_imu = Imu()
    msg_imu.header = msg
    msg_imu.orientation = msg_orient
    msg_imu.orientation_covariance = rospy.get_param('odometry_imu_node/orientation_covariance')
    msg_imu.angular_velocity = msg_gyro
    msg_imu.angular_velocity_covariance = rospy.get_param('odometry_imu_node/gyro_covariance')
    msg_imu.linear_acceleration = msg_accel
    msg_imu.linear_acceleration_covariance = rospy.get_param('odometry_imu_node/accel_covariance')
    pub.publish(msg_imu)  # publishing data to /imu/data


if __name__ == '__main__':
    seq = 0
    rospy.init_node("odometry_imu_node")

    sub = rospy.Subscriber("/imu/orient", Quaternion, imu_orient)
    sub = rospy.Subscriber("/imu/accel", Vector3, imu_accel)
    sub = rospy.Subscriber("/imu/gyro", Vector3, imu_gyro)
    sub = rospy.Subscriber("/imu/head", Header, imu_head)
    pub = rospy.Publisher("/imu/data", Imu, queue_size=10)

    rospy.spin()
