cmake_minimum_required(VERSION 2.8.3)
project(morus_msgs)

find_package(catkin REQUIRED message_generation std_msgs dynamic_reconfigure)

add_message_files(
  FILES
  MotorSpeed.msg
  PIDController.msg
  SMCStatusHeight.msg
  SMCStatusAttitude.msg
)

generate_messages(DEPENDENCIES std_msgs)


generate_dynamic_reconfigure_options(
cfg/MavCtlParams.cfg
cfg/MavZCtlParams.cfg
cfg/MavXYCtlParams.cfg
cfg/MavAttitudeCtlParams.cfg
cfg/MavPosCtlParams.cfg
cfg/SmcPosCtlParams.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

