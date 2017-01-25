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
    
    imuMsg_subscriber_ = this->rosnodeHandle_->subscribe(imu_topic_name_,1, &BaseAttitude2Motion::onImuMsg, this);

    std::stringstream ss(connected_ice_motors_);
    std::string token;
    while (ss >> token)
    {
        gmMsg_subscribers_.push_back(this->rosnodeHandle_->subscribe(token.c_str(),1, &BaseAttitude2Motion::onICEMsg, this));
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&BaseAttitude2Motion::onUpdate, this));
}

void BaseAttitude2Motion::onImuMsg(const sensor_msgs::ImuPtr& msg)
{
	ROS_INFO("Received IMU message");
}

void BaseAttitude2Motion::onICEMsg(const morus_uav_ros_msgs::GmStatusPtr& msg)
{
	ROS_INFO("Received ICE message");
}

void BaseAttitude2Motion::onUpdate()
{
    common::Time cur_time = this->world_->GetSimTime();

    // Calculate the wind force.

    math::Vector3 wind_gust(0, 500, 0);
    
    math::Vector3 xyz_offset(0, 0, 0); 
    // Calculate the wind gust force.
    //double wind_gust_strength = wind_gust_force_mean_;
    //wind_gust = wind_gust_strength * wind_gust_direction_;
    // Apply a force from the wind gust to the link.
    link_->AddForceAtRelativePosition(wind_gust, xyz_offset);

}
GZ_REGISTER_MODEL_PLUGIN(BaseAttitude2Motion);
