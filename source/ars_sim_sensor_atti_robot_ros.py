#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Header


import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped




import tf_conversions as tf

import tf2_ros



#
import ars_lib_helpers





class ArsSimSensorAttiRobotRos:

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

  def __init__(self):

    # Covariance on measurement of orientation
    self.cov_meas_att = {'x': 0.005, 'y': 0.005, 'z': 0.05}

    #
    self.flag_robot_pose_set = False
    self.robot_frame_id = ''
    self.robot_pose_timestamp = rospy.Time()
    self.robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()

    # Measurement sensor loop
    # freq
    self.meas_sens_loop_freq = 50.0
    # Timer
    self.meas_sens_loop_timer = None



    # end
    return


  def init(self, node_name='ars_sim_sensor_atti_robot_ros_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_sensors_robot')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###


    
    # End
    return


  def open(self):


    # Subscribers

    # 
    self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robotPoseCallback)


    # Publishers

    # 
    self.meas_robot_atti_pub = rospy.Publisher('meas_robot_attitude', QuaternionStamped, queue_size=1)


    # Timers
    #
    self.meas_sens_loop_timer = rospy.Timer(rospy.Duration(1.0/self.meas_sens_loop_freq), self.measSensorLoopTimerCallback)



    # End
    return


  def run(self):

    rospy.spin()

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



  def measSensorLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

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
    noise_atti_quat_tf = tf.transformations.quaternion_from_euler(noise_atti_ang[0], noise_atti_ang[1], noise_atti_ang[2], axes='sxyz')
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
