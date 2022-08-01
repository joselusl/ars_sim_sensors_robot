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
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped



import tf_conversions as tf

import tf2_ros



#
import ars_lib_helpers





class ArsSimSensorPosiRobotRos:

  #######

  # Covariance on measurement of position
  cov_meas_pos = None

  # Robot pose subscriber
  robot_pose_sub = None

  # Meas robot posi pub
  meas_robot_posi_pub = None


  # Robot Pose
  flag_robot_posi_set = False
  robot_frame_id = None
  robot_pose_timestamp = None
  robot_posi = None


  # Measurement sensor loop
  # freq
  meas_sens_loop_freq = None
  # Timer
  meas_sens_loop_timer = None


  


  #########

  def __init__(self):

    # Covariance on measurement of position
    self.cov_meas_pos = {'x': 0.05, 'y': 0.05, 'z': 0.05}

    #
    self.flag_robot_pose_set = False
    self.robot_frame_id = ''
    self.robot_pose_timestamp = rospy.Time()
    self.robot_posi = np.zeros((3,), dtype=float)

    # Measurement sensor loop
    # freq
    self.meas_sens_loop_freq = 1.0
    # Timer
    self.meas_sens_loop_timer = None



    # end
    return


  def init(self, node_name='ars_sim_sensor_posi_robot_ros_node'):
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
    self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robotPositionCallback)


    # Publishers

    # 
    self.meas_robot_posi_pub = rospy.Publisher('meas_robot_position', PointStamped, queue_size=1)


    # Timers
    #
    self.meas_sens_loop_timer = rospy.Timer(rospy.Duration(1.0/self.meas_sens_loop_freq), self.measSensorLoopTimerCallback)



    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotPositionCallback(self, robot_pose_msg):

    #
    self.flag_robot_pose_set = True

    self.robot_frame_id = robot_pose_msg.header.frame_id

    self.robot_pose_timestamp = robot_pose_msg.header.stamp

    # Position
    self.robot_posi[0] = robot_pose_msg.pose.position.x
    self.robot_posi[1] = robot_pose_msg.pose.position.y
    self.robot_posi[2] = robot_pose_msg.pose.position.z
   
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

    # Position
    meas_posi[0] = self.robot_posi[0] + np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_pos['x']))
    meas_posi[1] = self.robot_posi[1] + np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_pos['y']))
    meas_posi[2] = self.robot_posi[2] + np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_pos['z']))

    # Covariance
    #meas_cov_posi = np.diag([self.cov_meas_pos['x'], self.cov_meas_pos['y'], self.cov_meas_pos['z']])


    # Filling the message

    #
    meas_header_msg = Header()
    meas_robot_posi_msg = Point()
    meas_robot_posi_stamp_msg = PointStamped()

    #
    meas_header_msg.frame_id = self.robot_frame_id
    meas_header_msg.stamp = self.robot_pose_timestamp

    # Position
    meas_robot_posi_msg.x = meas_posi[0]
    meas_robot_posi_msg.y = meas_posi[1]
    meas_robot_posi_msg.z = meas_posi[2]


    #
    meas_robot_posi_stamp_msg.header = meas_header_msg
    meas_robot_posi_stamp_msg.point = meas_robot_posi_msg

    #
    self.meas_robot_posi_pub.publish(meas_robot_posi_stamp_msg)

    #
    return
