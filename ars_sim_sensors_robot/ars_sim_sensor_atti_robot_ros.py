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
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped

import tf_transformations

#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers



class ArsSimSensorAttiRobotRos(Node):

  #######

  # Covariance on measurement of orientation
  cov_meas_att = None


  # Robot pose subscriber
  robot_pose_sub = None

  # Meas robot atti pub
  meas_robot_atti_pub = None


  # Robot Pose
  flag_robot_pose_set = False
  robot_frame_id = None
  robot_pose_timestamp = None
  robot_atti_quat = None
  robot_atti_quat_simp = None


  # Measurement sensor loop
  # freq
  meas_sens_loop_freq = None
  # Timer
  meas_sens_loop_timer = None


  


  #########

  def __init__(self, node_name='ars_sim_sensor_atti_robot_ros_node'):
    # Init ROS
    super().__init__(node_name)

    # Covariance on measurement of orientation
    self.cov_meas_att = {'x': 0.005, 'y': 0.005, 'z': 0.05}

    #
    self.flag_robot_pose_set = False
    self.robot_frame_id = ''
    self.robot_pose_timestamp = Time()
    self.robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()

    # Measurement sensor loop
    # freq
    self.meas_sens_loop_freq = 50.0
    # Timer
    self.meas_sens_loop_timer = None

    #
    self.__init(node_name)

    # end
    return


  def __init(self, node_name='ars_sim_sensor_atti_robot_ros_node'):
    
    # Package path
    try:
      pkg_path = get_package_share_directory('ars_sim_sensors_robot')
      print(f"The path to the package is: {pkg_path}")
    except PackageNotFoundError:
      print("Package not found")
    

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
    self.meas_robot_atti_pub = self.create_publisher(QuaternionStamped, 'meas_robot_attitude', qos_profile=10)


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

    # Attitude
    noise_atti_ang = np.zeros((3,), dtype=float)
    noise_atti_ang[0] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_att['x']))
    noise_atti_ang[1] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_att['y']))
    noise_atti_ang[2] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_att['z']))
    noise_atti_quat_tf = tf_transformations.quaternion_from_euler(noise_atti_ang[0], noise_atti_ang[1], noise_atti_ang[2], axes='sxyz')
    noise_atti_quat = np.roll(noise_atti_quat_tf, 1)
    meas_atti_quat = ars_lib_helpers.Quaternion.quatProd(self.robot_atti_quat, noise_atti_quat)

    # Covariance
    #meas_cov_atti = np.diag(self.cov_meas_att['x'], self.cov_meas_att['y'], self.cov_meas_att['z']])


    # Filling the message

    #
    meas_header_msg = Header()
    meas_robot_atti_msg = Quaternion()
    meas_robot_atti_stamp_msg = QuaternionStamped()

    #
    meas_header_msg.frame_id = self.robot_frame_id
    meas_header_msg.stamp = self.robot_pose_timestamp

    # Attitude
    meas_robot_atti_msg.w = meas_atti_quat[0]
    meas_robot_atti_msg.x = meas_atti_quat[1]
    meas_robot_atti_msg.y = meas_atti_quat[2]
    meas_robot_atti_msg.z = meas_atti_quat[3]


    #
    meas_robot_atti_stamp_msg.header = meas_header_msg
    meas_robot_atti_stamp_msg.quaternion = meas_robot_atti_msg


    #
    self.meas_robot_atti_pub.publish(meas_robot_atti_stamp_msg)

    #
    return
