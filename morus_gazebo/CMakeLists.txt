cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
project(morus_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	roscpp
	std_msgs
	geometry_msgs
	genmsg
	sensor_msgs
	message_generation
	gazebo_ros
	gazebo_msgs
	morus_uav_ros_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
##########################
## Gazebo related stuff ##
##########################
if (FALSE)#${gazebo_VERSION_MAJOR} LESS 6
  include (FindPkgConfig)
  if (PKG_CONFIG_FOUND)
    pkg_check_modules(XML libxml-2.0)
    pkg_check_modules(OGRE OGRE)
    pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  else()
    message(FATAL_ERROR "pkg-config is required; please install it")
  endif()

  # Depend on system install of Gazebo
  
  find_package(Boost REQUIRED COMPONENTS thread)

  execute_process(COMMAND
    pkg-config --variable=plugindir OGRE
    OUTPUT_VARIABLE OGRE_PLUGIN_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()
find_package(gazebo REQUIRED)
find_package(Eigen REQUIRED)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS include launch plugins worlds ${Eigen_INCLUDE_DIRS}
	LIBRARIES morus_gazebo
	CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime sensor_msgs morus_uav_ros_msgs
	DEPENDS system_lib gazebo_ros eigen
	)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${Eigen_INCLUDE_DIRS})

## Declare a C++ library
# add_library(morus_gazebo
#   src/${PROJECT_NAME}/morus_gazebo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(morus_gazebo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(morus_gazebo_node src/morus_gazebo_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(morus_gazebo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(morus_gazebo_node
#   ${catkin_LIBRARIES}
# )

link_directories(${GAZEBO_LIBRARY_DIRS})# ${OGRE_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})# ${Boost_INCLUDE_DIR}  ${OGRE_INCLUDE_DIRS} ${OGRE-Terrain_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME}_gm_status plugins/gm_status/gmStatus.cpp)
target_link_libraries(${PROJECT_NAME}_gm_status ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_library(${PROJECT_NAME}_get_pose plugins/get_pose/get_pose.cpp)
#target_link_libraries(${PROJECT_NAME}_get_pose  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_battery_status plugins/battery_status/batteryStatus.cpp)
target_link_libraries(${PROJECT_NAME}_battery_status  ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(${PROJECT_NAME}_base_attitude2motion plugins/base_attitude2motion/baseAttitude2Motion.cpp)
target_link_libraries(${PROJECT_NAME}_base_attitude2motion  ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(ned_pose_publisher src/nodes/ned_pose_publisher.cpp)
target_link_libraries(ned_pose_publisher ${catkin_LIBRARIES})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS morus_gazebo morus_gazebo_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_morus_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)