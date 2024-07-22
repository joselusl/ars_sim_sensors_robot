#!/usr/bin/env python3

import rclpy

from ars_sim_sensors_robot.ars_sim_sensor_vel_robot_ros import ArsSimSensorVelRobotRos


def main(args=None):

  rclpy.init(args=args)

  ars_sim_sensor_vel_robot_ros = ArsSimSensorVelRobotRos()

  ars_sim_sensor_vel_robot_ros.open()

  try:
      ars_sim_sensor_vel_robot_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_sim_sensor_vel_robot_ros.destroy_node()
    rclpy.try_shutdown()

  return 0


''' MAIN '''
if __name__ == '__main__':

  main()
