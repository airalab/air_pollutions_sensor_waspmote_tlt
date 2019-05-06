cmake_minimum_required(VERSION 2.8.3)
project(air_pollutions_sensor_waspmote_tlt)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/sensor_node
    scripts/storage_node
    scripts/publisher_node
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/air_sensor.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
