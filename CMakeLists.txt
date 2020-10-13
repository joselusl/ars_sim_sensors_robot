cmake_minimum_required(VERSION 2.8.3)
project(ars_sim_sensors_robot)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
  DEPENDS
  CATKIN_DEPENDS rospy std_msgs geometry_msgs visualization_msgs tf_conversions tf2_ros python-rospkg
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
