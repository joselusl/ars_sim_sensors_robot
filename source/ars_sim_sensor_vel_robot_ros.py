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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovarianceStamped



import tf_conversions

import tf2_ros




#
import ars_lib_helpers





class ArsSimSensorVelRobotRos:

  #######

  # Covariance on measurement of linear vel
  cov_meas_vel_lin = None

  # Covariance on measurement of angular vel
  cov_meas_vel_ang = None

  # Robot velocity subscriber
  robot_velocity_sub = None

  # Meas robot velocity pub
  meas_robot_velocity_pub = None
  meas_robot_velocity_cov_pub = None


  # Robot Vel
  flag_robot_velocity_set = False
  robot_frame_id = ''
  robot_velocity_timestamp = rospy.Time()
  robot_posi = None
  robot_atti_quat_simp = None


  # Measurement sensor loop
  # freq
  meas_sens_loop_freq = None
  # Timer
  meas_sens_loop_timer = None
  


  #########

  def __init__(self):

    # Covariance on measurement of linear vel
    self.cov_meas_vel_lin = {'x': 0.01, 'y': 0.01, 'z': 0.01}

    # Covariance on measurement of angular vel
    self.cov_meas_vel_ang = {'x': 0.01, 'y': 0.01, 'z': 0.01}


    #
    self.flag_robot_velocity_set = False
    self.robot_frame_id = ''
    self.robot_velocity_timestamp = rospy.Time()
    self.robot_vel_lin = np.zeros((3,), dtype=float)
    self.robot_vel_ang = np.zeros((3,), dtype=float)


    # Measurement sensor loop
    # freq
    self.meas_sens_loop_freq = 25.0
    # Timer
    self.meas_sens_loop_timer = None


    # end
    return


  def init(self, node_name='ars_sim_sensor_vel_robot_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_sensor_vel_robot')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###


    
    # End
    return


  def open(self):


    # Subscribers

    # 
    self.robot_velocity_sub = rospy.Subscriber('robot_velocity', TwistStamped, self.robotVelocityCallback)
    

    # Publishers

    # 
    self.meas_robot_velocity_pub = rospy.Publisher('meas_robot_velocity', TwistStamped, queue_size=1)
    # 
    self.meas_robot_velocity_cov_pub = rospy.Publisher('meas_robot_velocity_cov', TwistWithCovarianceStamped, queue_size=1)


    # Timers
    #
    self.meas_sens_loop_timer = rospy.Timer(rospy.Duration(1.0/self.meas_sens_loop_freq), self.measSensorLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotVelocityCallback(self, robot_velocity_msg):

    #
    self.flag_robot_velocity_set = True

    #
    self.robot_frame_id = robot_velocity_msg.header.frame_id

    self.robot_velocity_timestamp = robot_velocity_msg.header.stamp

    #
    self.robot_vel_lin[0] = robot_velocity_msg.twist.linear.x
    self.robot_vel_lin[1] = robot_velocity_msg.twist.linear.y
    self.robot_vel_lin[2] = robot_velocity_msg.twist.linear.z

    #
    self.robot_vel_ang[0] = robot_velocity_msg.twist.angular.x
    self.robot_vel_ang[1] = robot_velocity_msg.twist.angular.y
    self.robot_vel_ang[2] = robot_velocity_msg.twist.angular.z

    
    #
    return


  def measSensorLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    if(self.flag_robot_velocity_set == False):
      return

    # Computing the measurement

    #
    meas_vel_lin = np.zeros((3,), dtype=float)
    meas_vel_ang = np.zeros((3,), dtype=float)

    #
    meas_vel_lin[0] = self.robot_vel_lin[0] + np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_vel_lin['x']))
    meas_vel_lin[1] = self.robot_vel_lin[1] + np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_vel_lin['y']))
    meas_vel_lin[2] = self.robot_vel_lin[2] + np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_vel_lin['z']))

    #
    meas_vel_ang[0] = self.robot_vel_ang[0] + np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_vel_ang['x']))
    meas_vel_ang[1] = self.robot_vel_ang[1] + np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_vel_ang['y']))
    meas_vel_ang[2] = self.robot_vel_ang[2] + np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_vel_ang['z']))

    # Covariance
    meas_cov_vel = np.diag([self.cov_meas_vel_lin['x'], self.cov_meas_vel_lin['y'], self.cov_meas_vel_lin['z'], self.cov_meas_vel_ang['x'], self.cov_meas_vel_ang['y'], self.cov_meas_vel_ang['z']])



    # Filling the message

    #
    meas_header_msg = Header()
    meas_robot_velocity_msg = Twist()
    meas_robot_velocity_stamp_msg = TwistStamped()
    meas_robot_velocity_stamp_cov_msg = TwistWithCovarianceStamped()

    #
    meas_header_msg.frame_id = self.robot_frame_id
    meas_header_msg.stamp = self.robot_velocity_timestamp

    # Linear
    meas_robot_velocity_msg.linear.x = meas_vel_lin[0]
    meas_robot_velocity_msg.linear.y = meas_vel_lin[1]
    meas_robot_velocity_msg.linear.z = meas_vel_lin[2]

    # Angular
    meas_robot_velocity_msg.angular.x = meas_vel_ang[0]
    meas_robot_velocity_msg.angular.y = meas_vel_ang[1]
    meas_robot_velocity_msg.angular.z = meas_vel_ang[2]

    #
    meas_robot_velocity_stamp_msg.header = meas_header_msg
    meas_robot_velocity_stamp_msg.twist = meas_robot_velocity_msg

    #
    meas_robot_velocity_stamp_cov_msg.header = meas_header_msg
    meas_robot_velocity_stamp_cov_msg.twist.covariance = meas_cov_vel.reshape((36,1))
    meas_robot_velocity_stamp_cov_msg.twist.twist = meas_robot_velocity_msg
    
    #
    self.meas_robot_velocity_pub.publish(meas_robot_velocity_stamp_msg)
    self.meas_robot_velocity_cov_pub.publish(meas_robot_velocity_stamp_cov_msg)

    #
    return
