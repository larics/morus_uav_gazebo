#include "morus_plugins/gazebo_ductedfan_motor_model.h"
#include <cmath>

namespace gazebo {

GazeboMotorModel::~GazeboMotorModel() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboMotorModel::InitializeParams() {}

void GazeboMotorModel::Publish() {
  turning_velocity_msg_.data = joint_->GetVelocity(0);
  motor_velocity_pub_.publish(turning_velocity_msg_);

  angle_control_flap_command_msg_.data = angle_control_flap_ref_;
  angle_control_flap_command_pub_.publish(angle_control_flap_command_msg_);  
  //std::cout << "angle_control_flap_pub1 " << angle_control_flap_command_msg_.data  << std::endl;

}

void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_ << "\".");


  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");


  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  }
  else
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";


  //parameters from morus_base.urdf.xacro
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedSubTopic", wind_speed_sub_topic_, wind_speed_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                           motor_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "angleControlFlapRefSubTopic", angle_control_flap_ref_sub_topic_,
                           angle_control_flap_ref_sub_topic_);
  getSdfParam<std::string>(_sdf, "angleControlFlapCommandPubTopic", angle_control_flap_command_pub_topic_,
                           angle_control_flap_command_pub_topic_);
  getSdfParam<std::string>(_sdf, "angleControlFlapValueSubTopic", angle_control_flap_value_sub_topic_,
                           angle_control_flap_value_sub_topic_);


  getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
                      rolling_moment_coefficient_);
  getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(_sdf, "momentConstant", moment_constant_, moment_constant_);

  getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  getSdfParam<double>(_sdf, "fluidDensity", fluid_density_, fluid_density_);
  getSdfParam<double>(_sdf, "areaControlFlap", area_control_flap_, area_control_flap_);
  getSdfParam<double>(_sdf, "areaAntitorqueFlap", area_antitorque_flap_, area_antitorque_flap_);
  getSdfParam<double>(_sdf, "distanceControlFlap", distance_control_flap_, distance_control_flap_);
  getSdfParam<double>(_sdf, "distanceAntitorqueFlap", distance_antitorque_flap_, distance_antitorque_flap_);

  getSdfParam<double>(_sdf, "thrustCoefficient", thrust_coefficient_, thrust_coefficient_);
  getSdfParam<double>(_sdf, "torqueCoefficient", torque_coefficient_, torque_coefficient_);
  getSdfParam<double>(_sdf, "slipVelocityCoefficient", slip_velocity_coefficient_, slip_velocity_coefficient_);

  getSdfParam<double>(_sdf, "liftCoefficientControlFlap", lift_coefficient_control_flap_, lift_coefficient_control_flap_);
  getSdfParam<double>(_sdf, "dragCoefficientControlFlap", drag_coefficient_control_flap_, drag_coefficient_control_flap_);
  getSdfParam<double>(_sdf, "liftCoefficientAntitorqueFlap", lift_coefficient_antitorque_flap_, lift_coefficient_antitorque_flap_);
  getSdfParam<double>(_sdf, "dragCoefficientAntitorqueFlap", drag_coefficient_antitorque_flap_, drag_coefficient_antitorque_flap_);

  getSdfParam<double>(_sdf, "liftCoefficientControlFlapAt0", lift_coefficient_control_flap_at0_, lift_coefficient_control_flap_at0_);
  getSdfParam<double>(_sdf, "dragCoefficientControlFlapAt0", drag_coefficient_control_flap_at0_, drag_coefficient_control_flap_at0_);
  getSdfParam<double>(_sdf, "liftCoefficientAntitorqueFlapAt0", lift_coefficient_antitorque_flap_at0_, lift_coefficient_antitorque_flap_at0_);
  getSdfParam<double>(_sdf, "dragCoefficientAntitorqueFlapAt0", drag_coefficient_antitorque_flap_at0_, drag_coefficient_antitorque_flap_at0_);


  //std::cout << "fluid density " <<fluid_density_ << std::endl;
  // std::cout << area_control_flap_ << std::endl;
  // std::cout << area_antitorque_flap_ << std::endl;
  // std::cout << distance_control_flap_ << std::endl;
  // std::cout << distance_antitorque_flap_ << std::endl;
  // std::cout << thrust_coefficient_ << std::endl;
  // std::cout << torque_coefficient_ << std::endl;
  // std::cout << slip_velocity_coefficient_ << std::endl;
  // std::cout << lift_coefficient_control_flap_ << std::endl;
  // std::cout << drag_coefficient_control_flap_ << std::endl;
  // std::cout << drag_coefficient_control_flap_at0_ << std::endl;
  // std::cout << lift_coefficient_antitorque_flap_at0_ << std::endl;
  // std::cout << drag_coefficient_antitorque_flap_at0_ << std::endl;

  //std::cout << "angle_control_flap_sub_topic_" << angle_control_flap_sub_topic_ << std::endl;  


  // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
#if GAZEBO_MAJOR_VERSION < 5
  joint_->SetMaxForce(0, max_force_);
#endif
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

  //Publishers and Subscribers
  command_sub_ = node_handle_->subscribe(command_sub_topic_, 1, &GazeboMotorModel::VelocityCallback, this);
  wind_speed_sub_ = node_handle_->subscribe(wind_speed_sub_topic_, 1, &GazeboMotorModel::WindSpeedCallback, this);
  motor_velocity_pub_ = node_handle_->advertise<std_msgs::Float32>(motor_speed_pub_topic_, 1);

  //angle_control_flap_sub__ subscribes to the outer reference values, basicly gui
  angle_control_flap_ref_sub_ = node_handle_->subscribe(angle_control_flap_ref_sub_topic_, 1, &GazeboMotorModel::AngleControlFlapRefCallback, this);
  //angle_control_flap_command_pub__ publishes values to the angle controllers  
  angle_control_flap_command_pub_ = node_handle_->advertise<std_msgs::Float64>(angle_control_flap_command_pub_topic_, 1);
  //angle_control_flap_value_sub_ subscribes to the actual process values (real angle value, not the reference)
  angle_control_flap_value_sub_ = node_handle_->subscribe(angle_control_flap_value_sub_topic_, 1, &GazeboMotorModel::AngleControlFlapValueCallback, this);
  
  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}

// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  Publish();
}

void GazeboMotorModel::VelocityCallback(const mav_msgs::ActuatorsConstPtr& rot_velocities) {
  ROS_ASSERT_MSG(rot_velocities->angular_velocities.size() > motor_number_,
                 "You tried to access index %d of the MotorSpeed message array which is of size %d.",
                 motor_number_, rot_velocities->angular_velocities.size());
  ref_motor_rot_vel_ = std::min(rot_velocities->angular_velocities[motor_number_], max_rot_velocity_);
}

void GazeboMotorModel::WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed) {
  // TODO(burrimi): Transform velocity to world frame if frame_id is set to something else.
  wind_speed_W_.x = wind_speed->velocity.x;
  wind_speed_W_.y = wind_speed->velocity.y;
  wind_speed_W_.z = wind_speed->velocity.z;
  //std::cout << "wind_speed 1 " << wind_speed_W_.x << std::endl;
}

void GazeboMotorModel::AngleControlFlapRefCallback(const morus_msgs::AngleControlFlapRefConstPtr& angle){
	angle_control_flap_ref_ = angle->anglecontrolflap;
	//std::cout << "angle_control_flap_ref_1 " << angle_control_flap_ref_ << std::endl;
}

void GazeboMotorModel::AngleControlFlapValueCallback(const control_msgs::JointControllerStatePtr& msg){
	angle_control_flap_ = msg->process_value;
	//std::cout << "angle_control_flap_value_1 " << angle_control_flap_ << std::endl;
}


void GazeboMotorModel::UpdateForcesAndMoments() {	
  motor_rot_vel_ = joint_->GetVelocity(0);
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double force = real_motor_velocity * real_motor_velocity * motor_constant_;
  // Apply a force to the link.
  //link_->AddRelativeForce(math::Vector3(0, 0, force));


  //Ducted fan formulas

  if (motor_number_ == 0 || motor_number_ == 2) { // we assume there is only one control flap wing beneath the rotor
    flag_x = 1;
    flag_y = 0;
  }
  else if (motor_number_ == 1 || motor_number_ == 3) {
    flag_x = 0;
    flag_y = 1;
  }
  else {
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";
  }

  if (angle_control_flap_ > 0.3 || angle_control_flap_ < -0.3){ // maximum angle value is 15 deg (0.26179 rad)
  	angle_control_flap_ = 0; // values before the morus_control.launch are large and incorrect and cause problems with forces
  }


  angle_antitorque_flap_ = 0; //antitorque flaps for single rotor vehicles can be added easily, decided not to use them
  double slip_velocity_squared_ = real_motor_velocity * real_motor_velocity * slip_velocity_coefficient_;
  

  double force_x_ = fluid_density_ * area_control_flap_ * slip_velocity_squared_ * 
  lift_coefficient_control_flap_ * angle_control_flap_ * flag_x;
  double force_y_ = fluid_density_ * area_control_flap_ * slip_velocity_squared_ * 
  lift_coefficient_control_flap_ * angle_control_flap_ * flag_y;
  
  double force_antitorque_flap_ = fluid_density_ * area_antitorque_flap_ * slip_velocity_squared_ * 
  (drag_coefficient_antitorque_flap_ * angle_antitorque_flap_ * angle_antitorque_flap_ + drag_coefficient_antitorque_flap_at0_);
  double force_thrust_ = real_motor_velocity * real_motor_velocity * thrust_coefficient_;
  double force_control_flap_ = fluid_density_ * area_control_flap_ * slip_velocity_squared_ * 
  (drag_coefficient_control_flap_ * angle_control_flap_ * angle_control_flap_ + drag_coefficient_control_flap_at0_);
  double force_z_ = force_thrust_ - force_antitorque_flap_ - force_control_flap_;

  
  double moment_x_ = force_x_ * distance_control_flap_ ;
  double moment_y_ = force_y_ * distance_control_flap_ ;
  double moment_z_1 = - turning_direction_ * torque_coefficient_ * force_thrust_;
  double moment_z_2 = fluid_density_ * area_antitorque_flap_ * slip_velocity_squared_*
  (lift_coefficient_antitorque_flap_ * angle_antitorque_flap_ + lift_coefficient_antitorque_flap_at0_);
  double moment_z_ = - turning_direction_ * torque_coefficient_ * force_thrust_ + fluid_density_ * area_antitorque_flap_ * slip_velocity_squared_* 
  (lift_coefficient_antitorque_flap_ * angle_antitorque_flap_ + lift_coefficient_antitorque_flap_at0_);
  //decided that the antitorque part of the formulas is not used

  link_->AddForce(math::Vector3(force_x_, force_y_, force_z_));




  // Forces from Philppe Martin's and Erwan Salaün's
  // 2010 IEEE Conference on Robotics and Automation paper
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // - \omega * \lambda_1 * V_A^{\perp}
  math::Vector3 joint_axis = joint_->GetGlobalAxis(0);
  math::Vector3 body_velocity_W = link_->GetWorldLinearVel();
  math::Vector3 relative_wind_velocity_W = body_velocity_W - wind_speed_W_;
  math::Vector3 body_velocity_perpendicular = relative_wind_velocity_W - (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);
  math::Vector3 air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * body_velocity_perpendicular;
  // Apply air_drag to link.
  link_->AddForce(air_drag);
  // Moments
  // Getting the parent link, such that the resulting torques can be applied to it.
  physics::Link_V parent_links = link_->GetParentJointsLinks();
  // The tansformation from the parent_link to the link_.
  math::Pose pose_difference = link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose();


  math::Vector3 drag_torque(moment_x_, moment_y_, moment_z_1);


  // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
  math::Vector3 drag_torque_parent_frame = pose_difference.rot.RotateVector(drag_torque);
  parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

  math::Vector3 rolling_moment;
  // - \omega * \mu_1 * V_A^{\perp}
  rolling_moment = -std::abs(real_motor_velocity) * rolling_moment_coefficient_ * body_velocity_perpendicular;
  parent_links.at(0)->AddTorque(rolling_moment);
  // Apply the filter on the motor's velocity.
  double ref_motor_rot_vel;
  ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);
  joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);

  
  //std::cout << "angle_control_flap_2 " << angle_control_flap_ << std::endl;
  //std::cout << "angle_control_flap_sub_topic_" << angle_control_flap_sub_topic_ << std::endl; 

  //std::cout << "motor_number_ " << motor_number_ << std::endl;
  //std::cout << "force " << force << std::endl;
  /*std::cout << "force_x_ " << force_x_ << std::endl; 
  std::cout << "force_y_ " << force_y_ << std::endl;
  std::cout << "force_z_ " << force_z_ << std::endl;
  std::cout << "slip_velocity_squared_ " << slip_velocity_squared_ << std::endl;
  std::cout << "force_thrust_" << force_thrust_ << std::endl;
  std::cout << "force_control_flap_ " << force_control_flap_ << std::endl;
  std::cout << "force_antitorque_flap_ " << force_antitorque_flap_ << std::endl;

  std::cout << "drag_torque " << drag_torque << std::endl;
  std::cout << "rolling_moment " << rolling_moment << std::endl;
  std::cout << "moment_x_ " << moment_x_ << std::endl;
  std::cout << "moment_y_ " << moment_y_ << std::endl;
  std::cout << "moment_z_ " << moment_z_ << std::endl;
  std::cout << "moment_z_1 " << moment_z_1 << std::endl;
  std::cout << "moment_z_2 " << moment_z_2 << std::endl << std::endl;*/
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}


