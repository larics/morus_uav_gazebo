cmake_minimum_required(VERSION 2.8.3)
project(morus_plugins)


add_definitions(-std=c++11)


# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  mav_msgs
  morus_msgs
  octomap_msgs
  octomap_ros
  rosbag
  roscpp
  rotors_comm
  rotors_control
  std_srvs
  tf
)

find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES morus_gazebo_ductedfan_motor_model
  CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs octomap_msgs octomap_ros rosbag roscpp rotors_comm rotors_control std_srvs tf
  DEPENDS eigen gazebo octomap opencv
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_library(morus_gazebo_ductedfan_motor_model src/gazebo_ductedfan_motor_model.cpp)
target_link_libraries(morus_gazebo_ductedfan_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(morus_gazebo_ductedfan_motor_model ${catkin_EXPORTED_TARGETS})


install(
  TARGETS
    morus_gazebo_ductedfan_motor_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

