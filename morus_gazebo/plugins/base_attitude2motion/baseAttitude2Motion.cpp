#include <morus_gazebo/baseAttitude2Motion.h>

#include <string>

using namespace gazebo;

BaseAttitude2Motion::BaseAttitude2Motion()
{

}

BaseAttitude2Motion::~BaseAttitude2Motion()
{
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    if (this->rosnodeHandle_)
    { 
        this->rosnodeHandle_->shutdown();
    }

    delete this->rosnodeHandle_;
}

void BaseAttitude2Motion::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Keep the information about the model
    model_ = _parent;
    world_ = model_->GetWorld();

    this->robot_namespace_ = "";
    if(_sdf->HasElement("robotNamespace")) this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else ROS_INFO("BaseAttitude2Motion plugin missing <robotNamespace>, defaults to \"%s\"", robot_namespace_.c_str());
    
    this->imu_topic_name_ = "imu";
    if(_sdf->HasElement("imuTopicName")) this->imu_topic_name_ = _sdf->GetElement("imuTopicName")->Get<std::string>();
    else ROS_INFO("BaseAttitude2Motion plugin missing <imuTopicName>, defaults to \"%s\\%s\"",robot_namespace_.c_str(), imu_topic_name_.c_str());
    
    this->connected_ice_motors_ = "gm1 gm2 gm3 gm4";;
    if(_sdf->HasElement("connectedICEs")) this->connected_ice_motors_ = _sdf->GetElement("connectedICEs")->Get<std::string>();
    else ROS_INFO("BaseAttitude2Motion plugin missing <connectedICEs>, defaults to \"%s\"", connected_ice_motors_.c_str());
    
    this->link_name_ = "base_link";
    if(_sdf->HasElement("linkName")) this->link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else ROS_INFO("BaseAttitude2Motion plugin missing <linkName>, defaults to \"%s\"",link_name_.c_str());
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL) gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");
    
    // Ensure that ROS has been initialized
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("baseAttitude2Motion (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libmorus_gazebo_base_attitude2motion.so' in the morus_gazebo package)");
      return;
    }
   

    this->rosnodeHandle_ = new ros::NodeHandle(this->robot_namespace_);
    
    imuMsg_subscriber_ = this->rosnodeHandle_->subscribe(imu_topic_name_,1,&BaseAttitude2Motion::onImuMsg,this);


    std::stringstream ss(connected_ice_motors_);
    std::string token;
    while (ss >> token)
    {
	// TO DO - boost::bind -> to add a distinguishable parameter for the callback
        gmMsg_subscribers_.push_back(this->rosnodeHandle_->subscribe(token.c_str(),1,&BaseAttitude2Motion::onIceMsg,this));//boost::bind(&BaseAttitude2Motion::onIceMsg, this, _1)));//,_1,token)));
	gmMsg_thrust_.push_back(0.0);
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&BaseAttitude2Motion::onUpdate, this));
}

void BaseAttitude2Motion::onImuMsg(const sensor_msgs::ImuPtr& msg)
{
  this->omega_body_frame_.x = msg->angular_velocity.x;
  this->omega_body_frame_.y = msg->angular_velocity.y;
  this->omega_body_frame_.z = msg->angular_velocity.z;
  //ROS_INFO("Received IMU message");
}

void BaseAttitude2Motion::onIceMsg(const morus_uav_ros_msgs::GmStatusPtr& msg)//,const std::string& topic) //const morus_uav_ros_msgs::GmStatusPtr& msg)
{
  int id = msg->motor_id-1;
  // Try to access and updated the vector of Forces with correct ID
  try {
    gmMsg_thrust_[id] = msg->force_M;     // vector::at [id] might not exist
  }
  catch (const std::out_of_range& oor) {
    //std::cerr << "Out of Range error: " << oor.what() << '\n';
    ROS_ERROR("Failed to access force at motor ID = %n. Supported IDs include 1-front, 2-right, 3-back, 4-left",&id);
  }
  //ROS_INFO("Received ICE message");
}

void BaseAttitude2Motion::onUpdate()
{
    common::Time cur_time = this->world_->GetSimTime();
    // Apply the body rotation based on IMU rotation speed Measurement
    // Angular rotation is applied in the body reference frame 
    this->model_->SetAngularVel(this->omega_body_frame_);

    // Calculate the sum of thrust forces
    double sum_of_forces = std::accumulate(gmMsg_thrust_.begin(), gmMsg_thrust_.end(), 0.0);
    // Project the sum of thrust forces in body z-axis
    math::Vector3 total_thrust(0.0, 0.0, sum_of_forces);
    
    math::Vector3 xyz_offset(0, 0, 0); 
    // Calculate the wind gust force.
    //double wind_gust_strength = wind_gust_force_mean_;
    //wind_gust = wind_gust_strength * wind_gust_direction_;
    // Apply a force from the wind gust to the link.
    link_->AddRelativeForce(total_thrust);

}
GZ_REGISTER_MODEL_PLUGIN(BaseAttitude2Motion);
