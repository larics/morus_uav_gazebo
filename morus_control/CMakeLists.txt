cmake_minimum_required(VERSION 2.8.3)
project(morus_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    rosgraph_msgs
    genmsg
    dynamic_reconfigure
)

find_package(cmake_modules REQUIRED)

generate_dynamic_reconfigure_options(
    # VPC MMCUAV parameters
    config/VpcMmcuavZCtlParams.cfg
    config/VpcMmcuavPositionCtlParams.cfg
    config/VpcMmcuavAttitudeCtlParams.cfg
    config/PositionCtlParams.cfg
    config/SmcMmcuavPositionCtlParams.cfg
    )

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

