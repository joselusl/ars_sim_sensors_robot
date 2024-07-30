#!/usr/bin/env python

import numpy as np
from numpy import *

import os


# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion

import tf_transformations

#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers



class ArsSimSensorPosRobotRos(Node):

  #######

  # Covariance on measurement of position
  cov_meas_pos = None

  # Covariance on measurement of orientation
  cov_meas_att = None


  # Robot pose subscriber
  robot_pose_sub = None

  # Meas robot pose pub
  meas_robot_pose_pub = None
  meas_robot_pose_cov_pub = None


  # Robot Pose
  flag_robot_pose_set = False
  robot_frame_id = None
  robot_pose_timestamp = None
  robot_posi = None
  robot_atti_quat = None
  robot_atti_quat_simp = None


  # Measurement sensor loop
  # freq
  meas_sens_loop_freq = None
  # Timer
  meas_sens_loop_timer = None


  


  #########

  def __init__(self, node_name='ars_sim_sensor_pos_robot_ros_node'):
    # Init ROS
    super().__init__(node_name)

    # Covariance on measurement of position
    self.cov_meas_pos = {'x': 0.05, 'y': 0.05, 'z': 0.05}

    # Covariance on measurement of orientation
    self.cov_meas_att = {'x': 0.005, 'y': 0.005, 'z': 0.05}

    #
    self.flag_robot_pose_set = False
    self.robot_frame_id = ''
    self.robot_pose_timestamp = Time()
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()

    # Measurement sensor loop
    # freq
    self.meas_sens_loop_freq = 1.0
    # Timer
    self.meas_sens_loop_timer = None

    #
    self.__init(node_name)

    # end
    return


  def __init(self, node_name='ars_sim_sensor_pos_robot_ros_node'):
    
    # Package path
    try:
      pkg_path = get_package_share_directory('ars_sim_sensors_robot')
      self.get_logger().info(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      self.get_logger().info("Package not found")

    
    #### READING PARAMETERS ###
    
    # 

    ###
    
    # End
    return


  def open(self):


    # Subscribers

    # 
    self.robot_pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.robotPoseCallback, qos_profile=10)


    # Publishers

    # 
    self.meas_robot_pose_pub = self.create_publisher(PoseStamped, 'meas_robot_pose', qos_profile=10)
    # 
    self.meas_robot_pose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, 'meas_robot_pose_cov', qos_profile=10)


    # Timers
    #
    self.meas_sens_loop_timer = self.create_timer(1.0/self.meas_sens_loop_freq, self.measSensorLoopTimerCallback)



    # End
    return


  def run(self):

    rclpy.spin(self)

    return


  def robotPoseCallback(self, robot_pose_msg):

    #
    self.flag_robot_pose_set = True

    self.robot_frame_id = robot_pose_msg.header.frame_id

    self.robot_pose_timestamp = robot_pose_msg.header.stamp

    # Position
    self.robot_posi[0] = robot_pose_msg.pose.position.x
    self.robot_posi[1] = robot_pose_msg.pose.position.y
    self.robot_posi[2] = robot_pose_msg.pose.position.z

    # Attitude quat
    self.robot_atti_quat[0] = robot_pose_msg.pose.orientation.w
    self.robot_atti_quat[1] = robot_pose_msg.pose.orientation.x
    self.robot_atti_quat[2] = robot_pose_msg.pose.orientation.y
    self.robot_atti_quat[3] = robot_pose_msg.pose.orientation.z
    # Attitude quat simp
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(self.robot_atti_quat)

    
    #
    return



  def measSensorLoopTimerCallback(self):

    #
    if(self.flag_robot_pose_set == False):
      return


    # Computing the measurement

    #
    meas_posi = np.zeros((3,), dtype=float)
    meas_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()

    # Position
    meas_posi[0] = self.robot_posi[0] + np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_pos['x']))
    meas_posi[1] = self.robot_posi[1] + np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_pos['y']))
    meas_posi[2] = self.robot_posi[2] + np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_pos['z']))

    # Attitude
    noise_atti_ang = np.zeros((3,), dtype=float)
    noise_atti_ang[0] = np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_att['x']))
    noise_atti_ang[1] = np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_att['y']))
    noise_atti_ang[2] = np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_att['z']))
    noise_atti_quat_tf = tf_transformations.quaternion_from_euler(noise_atti_ang[0], noise_atti_ang[1], noise_atti_ang[2], axes='sxyz')
    noise_atti_quat = np.roll(noise_atti_quat_tf, 1)
    meas_atti_quat = ars_lib_helpers.Quaternion.quatProd(self.robot_atti_quat, noise_atti_quat)

    # Covariance
    meas_cov_posi = np.diag([self.cov_meas_pos['x'], self.cov_meas_pos['y'], self.cov_meas_pos['z'], self.cov_meas_att['x'], self.cov_meas_att['y'], self.cov_meas_att['z']])


    # Filling the message

    #
    meas_header_msg = Header()
    meas_robot_pose_msg = Pose()
    meas_robot_pose_stamp_msg = PoseStamped()
    meas_robot_pose_cov_msg = PoseWithCovarianceStamped()

    #
    meas_header_msg.frame_id = self.robot_frame_id
    meas_header_msg.stamp = self.robot_pose_timestamp

    # Position
    meas_robot_pose_msg.position.x = meas_posi[0]
    meas_robot_pose_msg.position.y = meas_posi[1]
    meas_robot_pose_msg.position.z = meas_posi[2]


    # Attitude
    meas_robot_pose_msg.orientation.w = meas_atti_quat[0]
    meas_robot_pose_msg.orientation.x = meas_atti_quat[1]
    meas_robot_pose_msg.orientation.y = meas_atti_quat[2]
    meas_robot_pose_msg.orientation.z = meas_atti_quat[3]


    #
    meas_robot_pose_stamp_msg.header = meas_header_msg
    meas_robot_pose_stamp_msg.pose = meas_robot_pose_msg

    #
    meas_robot_pose_cov_msg.header = meas_header_msg
    meas_robot_pose_cov_msg.pose.covariance = meas_cov_posi.reshape((36,))
    meas_robot_pose_cov_msg.pose.pose = meas_robot_pose_msg

    #
    self.meas_robot_pose_pub.publish(meas_robot_pose_stamp_msg)
    self.meas_robot_pose_cov_pub.publish(meas_robot_pose_cov_msg)

    #
    return
