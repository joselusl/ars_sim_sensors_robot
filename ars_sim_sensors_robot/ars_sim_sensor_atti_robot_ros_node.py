#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_sim_sensor_atti_robot_ros import *




def main():

  ars_sim_sensor_atti_robot_ros = ArsSimSensorAttiRobotRos()

  ars_sim_sensor_atti_robot_ros.init()
  ars_sim_sensor_atti_robot_ros.open()

  try:
    ars_sim_sensor_atti_robot_ros.run()
  except rospy.ROSInterruptException:
    pass


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()