cmake_minimum_required(VERSION 2.8.3)
project(air_pollutions_sensor_waspmote_tlt)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    genmsg
    rospy
)

catkin_python_setup()

add_message_files(
  FILES
  Measurement.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/sensor_server
    scripts/sensor_node
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
