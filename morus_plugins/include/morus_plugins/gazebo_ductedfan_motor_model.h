


#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/default_topics.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rotors_comm/WindSpeed.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <morus_msgs/AngleControlFlapRef.h> //morus_msgs must be included in CMakeLists.txt, check all CMakeLists.txt
#include <morus_msgs/AngleControlFlapValue.h>
#include <control_msgs/JointControllerState.h>

#include "common.h"
#include "motor_model.hpp"

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {
// Default values
static const std::string kDefaultCommandSubTopic = "gazebo/command/motor_speed";
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";
static const std::string kDefaultAngleflapSubTopic = "morus/angle";

// Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
static constexpr double kDefaultMotorConstant = 8.54858e-06;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaulMaxRotVelocity = 838.0;
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;

class GazeboMotorModel : public MotorModel, public ModelPlugin {
 public:
  GazeboMotorModel()
      : ModelPlugin(),
        MotorModel(),
        command_sub_topic_(kDefaultCommandSubTopic),
        wind_speed_sub_topic_(kDefaultWindSpeedSubTopic),
        angle_control_flap_ref_sub_topic_(kDefaultAngleflapSubTopic),
        motor_speed_pub_topic_(mav_msgs::default_topics::MOTOR_MEASUREMENT),
        motor_number_(0),
        turning_direction_(turning_direction::CW),
        max_force_(kDefaultMaxForce),
        max_rot_velocity_(kDefaulMaxRotVelocity),
        moment_constant_(kDefaultMomentConstant),
        motor_constant_(kDefaultMotorConstant),
        ref_motor_rot_vel_(0.0),
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        time_constant_down_(kDefaultTimeConstantDown),
        time_constant_up_(kDefaultTimeConstantUp),
        node_handle_(nullptr),
        wind_speed_W_(0, 0, 0) {}

  virtual ~GazeboMotorModel();

  virtual void InitializeParams();
  virtual void Publish();

 protected:
  virtual void UpdateForcesAndMoments();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string command_sub_topic_;
  std::string wind_speed_sub_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  std::string angle_control_flap_ref_sub_topic_;
  std::string angle_control_flap_command_pub_topic_;
  std::string angle_control_flap_value_sub_topic_;

  int motor_number_;
  int turning_direction_;
  int flag_x;
  int flag_y;

  double max_force_;
  double max_rot_velocity_;
  double moment_constant_;
  double motor_constant_;
  double ref_motor_rot_vel_;
  double rolling_moment_coefficient_;
  double rotor_drag_coefficient_;
  double rotor_velocity_slowdown_sim_;
  double time_constant_down_;
  double time_constant_up_;

  double fluid_density_;
  double area_control_flap_;
  double area_antitorque_flap_;
  double distance_control_flap_;
  double distance_antitorque_flap_;
  double thrust_coefficient_;
  double torque_coefficient_;
  double slip_velocity_coefficient_;
  double lift_coefficient_control_flap_;
  double drag_coefficient_control_flap_;
  double lift_coefficient_antitorque_flap_;
  double drag_coefficient_antitorque_flap_;
  double lift_coefficient_control_flap_at0_;
  double drag_coefficient_control_flap_at0_;
  double lift_coefficient_antitorque_flap_at0_;
  double drag_coefficient_antitorque_flap_at0_;
  double angle_control_flap_;
  double angle_antitorque_flap_;
  double angle_control_flap_ref_;

  ros::NodeHandle* node_handle_;
  ros::Publisher motor_velocity_pub_;
  ros::Subscriber command_sub_;
  ros::Subscriber wind_speed_sub_;

  ros::Subscriber angle_control_flap_ref_sub_;
  ros::Publisher angle_control_flap_command_pub_;
  ros::Subscriber angle_control_flap_value_sub_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();

  std_msgs::Float32 turning_velocity_msg_;  
  std_msgs::Float64 angle_control_flap_command_msg_; //controller pub msg is float64, 
                                                     //controller sub msg is control_msgs/JointControllerState

  void VelocityCallback(const mav_msgs::ActuatorsConstPtr& rot_velocities);
  void WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed);
  void AngleControlFlapRefCallback(const morus_msgs::AngleControlFlapRefConstPtr& angle);
  void AngleControlFlapValueCallback(const control_msgs::JointControllerStatePtr& msg);


  std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
  math::Vector3 wind_speed_W_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H