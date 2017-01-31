#include <morus_gazebo/gmStatus.h>

#include <string>

using namespace gazebo;

GMStatus::GMStatus()
{

}

GMStatus::~GMStatus()
{
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    if (this->rosnode_)
    { 
        this->rosnode_->shutdown();
    }

    delete this->rosnode_;
}

void GMStatus::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    model = _parent;
    world = model->GetWorld();
    prev_sim_time_ = 0.0;
    
    this->robot_namespace_ = "";
    if(_sdf->HasElement("robotNamespace")) this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else ROS_INFO("GMStatus plugin missing <robotNamespace>, defaults to \"%s\"", robot_namespace_.c_str());
    
    this->gm_ID_ ="";
    if(_sdf->HasElement("gmID")) 
    {
      this->gm_ID_ = _sdf->GetElement("gmID")->Get<std::string>();
      this->gm_topic_ = this->model->GetName() + "/gm/"+ this->gm_ID_+ "/Status";
    }
    else
    {
      ROS_INFO("GMStatus plugin missing <gmID>, defaults to 0");
      this->gm_topic_ = this->model->GetName() + "/gm/0/Status";
    }
    
    this->link_name_ = "base_link";
    if(_sdf->HasElement("linkName")) this->link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else ROS_INFO("BaseAttitude2Motion plugin missing <linkName>, defaults to \"%s\"",link_name_.c_str());
    link_ = model->GetLink(link_name_);
    if (link_ == NULL) gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");
    
    if (_sdf->HasElement("jointName"))
      joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
    else
      gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
    // Get the pointer to the joint.
    joint_ = model->GetJoint(joint_name_);
    if (joint_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_ << "\".");
    
    
    if (_sdf->HasElement("motorConstant"))
    {
      motor_constant_= _sdf->GetElement("motorConstant")->Get<double>();
      motor_constant_=0.0004568;
    }
    else
      motor_constant_=0.0004568;
    //getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
    // Ensure that ROS has been initialized
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("GMStatusPlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
    
    // Subscriber
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<morus_uav_ros_msgs::GmIgnition>("morus/GmIgnition", 1,
											  boost::bind(&GMStatus::OnGMIgnitionMsg, this, _1),
											ros::VoidPtr(), &callback_queue_);
    OnGMIgnitionMsg_subscriber_ = this->rosnode_->subscribe(ops);

    // Publisher
    if (!gm_topic_.empty())
    {
        ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<morus_uav_ros_msgs::GmStatus>(
            gm_topic_, 1,
            ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
        gm_publisher_ = this->rosnode_->advertise(ops);
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GMStatus::OnUpdate, this));
}

void GMStatus::OnGMIgnitionMsg(const morus_uav_ros_msgs::GmIgnitionConstPtr& msg)
{
    // TO DO : Provide some action when Ignition is received
    // for now, just push notification 
    ROS_INFO("Received GMIgnition message");
}


void GMStatus::OnUpdate()
{
  // This function is called every Gazebo update, to update ROS msgs with the information
  // later published through ROS
  
  // First we need to measure DT
  common::Time cur_time = this->world->GetSimTime();
  sampling_time_ = cur_time.Double() - prev_sim_time_;
  prev_sim_time_ = cur_time.Double();
  // We measure the rotation of the rotor Gazebo joint
  double motor_rot_vel_ = joint_->GetVelocity(0);
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }
  double real_motor_velocity = motor_rot_vel_ * 10;//RotorVelocitySlowdownSim := 10 in rotors_sim
  // Standard thrust force calculation like the one produced in rotors_sim
  double force = real_motor_velocity * real_motor_velocity * motor_constant_;
  
  // Get new commands/state
  callback_queue_.callAvailable();
  
  // Here we fill the gm_status msg fith data
  this->gm_status_msg.header.frame_id = this->model->GetName();
  this->gm_status_msg.header.stamp.sec = cur_time.sec;
  this->gm_status_msg.header.stamp.nsec = cur_time.nsec;
  this->gm_status_msg.motor_id = atoi(this->gm_ID_.c_str());
  this->gm_status_msg.can_timestamp = 1;
  
  // For some reason one cannot simple gather force on rotor link
  // TO DO: Check why this is not working
  //math::Vector3 force_on_rotor = this->link_->GetRelativeForce();
  
  this->gm_status_msg.force_M = force;// force_on_rotor.z; // measured force in Newton

  this->gm_status_msg.speed_M	=2.0;	        	// measured rotational velocity radian per second
  this->gm_status_msg.temperature_M = 35.5;	    	// measured temperature in Celsius degree
  this->gm_status_msg.fuel_level_M = 0.1; 	    	// measured fuel level in percentage

  this->gm_status_msg.ignition_S = 1;             	// setted ignition flag (0-off, 1-on)
  this->gm_status_msg.starter_ppm_S  = 1000;        	// setted ppm value for e-starter (range 1000-1900)
  this->gm_status_msg.speed_S = 1300;		        // setted reference for gm velocity in radian per second
  
  
  gm_publisher_.publish(gm_status_msg);		// publish status msg to ROS


}
GZ_REGISTER_MODEL_PLUGIN(GMStatus);
