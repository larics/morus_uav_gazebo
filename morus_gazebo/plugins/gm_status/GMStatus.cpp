#include <morus_gazebo/GMStatus.h>

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

    this->robot_namespace_ = "";
    if(_sdf->HasElement("robotNamespace")) this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else ROS_INFO("GMStatus plugin missing <robotNamespace>, defaults to \"%s\"", robot_namespace_.c_str());
    
    this->gm_ID_ ="";
    if(_sdf->HasElement("gmID")) 
    {
      this->gm_ID_ = _sdf->GetElement("gmID")->Get<std::string>();
      this->gm_topic_ = this->model->GetName() + "/gm/"+ this->gm_ID_+ "/Status";
      this->gm_ignition_topic_ = this->model->GetName() + "/gm/"+ this->gm_ID_+ "/GmIgnition";
    }
    else
    {
      ROS_INFO("GMStatus plugin missing <gmID>, defaults to 0");
      this->gm_topic_ = this->model->GetName() + "/gm/0/Status";
    }
    
    if(_sdf->HasElement("FuelLevelTimeout")) this->fuel_level_timeout_ = _sdf->GetElement("FuelLevelTimeout")->Get<float>();
    else this->fuel_level_timeout_ = 10.0;

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
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<morus_uav_ros_msgs::GmIgnition>(this->gm_ignition_topic_, 1,
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
    
    // Initialize everything
    this->ignition_S = 0;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GMStatus::OnUpdate, this));
}

void GMStatus::OnGMIgnitionMsg(const morus_uav_ros_msgs::GmIgnitionConstPtr& msg)
{
	ROS_INFO("Received GMIgnition message");
	this->ignition_S = msg->ignition;
}


void GMStatus::OnUpdate()
{
    common::Time cur_time = this->world->GetSimTime();

    // Get new commands/state
    callback_queue_.callAvailable();

    this->gm_status_msg.header.frame_id = this->model->GetName();
    this->gm_status_msg.header.stamp.sec = cur_time.sec;
    this->gm_status_msg.header.stamp.nsec = cur_time.nsec;

    this->gm_status_msg.can_timestamp = 1;
    this->gm_status_msg.force_M = 1.0;		        // measured force in Newton
    this->gm_status_msg.speed_M	=2.0;	        	// measured rotational velocity radian per second
    this->gm_status_msg.temperature_M = 35.5;	    	// measured temperature in Celsius degree
    this->gm_status_msg.fuel_level_M = exp(-cur_time.sec/this->fuel_level_timeout_); 	    	// measured fuel level in percentage

    this->gm_status_msg.ignition_S = this->ignition_S;             	// setted ignition flag (0-off, 1-on)
    this->gm_status_msg.starter_ppm_S  = 1000;        	// setted ppm value for e-starter (range 1000-1900)
    this->gm_status_msg.speed_S = 1300;		        // setted reference for gm velocity in radian per second
    
    

    gm_publisher_.publish(gm_status_msg);		// publish status msg to ROS


}
GZ_REGISTER_MODEL_PLUGIN(GMStatus);
