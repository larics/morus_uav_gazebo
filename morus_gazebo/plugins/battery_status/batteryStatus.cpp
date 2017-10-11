#include <morus_gazebo/batteryStatus.h>

#include <string>

using namespace gazebo;

BatteryStatus::BatteryStatus()
{

}

BatteryStatus::~BatteryStatus()
{
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    if (this->rosnode_)
    { 
        this->rosnode_->shutdown();
    }

    delete this->rosnode_;
}

void BatteryStatus::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    model = _parent;
    world = model->GetWorld();

    this->robot_namespace_ = "";
    if(_sdf->HasElement("robotNamespace")) this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else ROS_INFO("BatteryStatus plugin missing <robotNamespace>, defaults to \"%s\"", robot_namespace_.c_str());
    
    this->battery_ID_ ="";
    if(_sdf->HasElement("batteryID")) 
    {
      this->battery_ID_ = _sdf->GetElement("batteryID")->Get<std::string>();
      this->battery_topic_ = this->model->GetName() + "/battery/"+ this->battery_ID_+ "/Status";
    }
    else
    {
      ROS_INFO("BatteryStatus plugin missing <batteryID>, defaults to 0");
      this->battery_topic_ = this->model->GetName() + "/battery/0/Status";
    }

    //if (!_sdf->HasElement("batteryTopic"))
    //{
    //    ROS_WARN("BatteryStatus plugin (ns = %s) missing <batteryTopic>, defaults to \"%s\"", robot_namespace_.c_str(), this->battery_topic_.c_str());
    //}
    //else this->battery_topic_ = _sdf->GetElement("batteryTopic")->Get<std::string>();

    // Ensure that ROS has been initialized
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("BatteryStatusPlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    // Publisher
    if (!battery_topic_.empty())
    {
        ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<morus_uav_ros_msgs::BatteryStatus>(
            battery_topic_, 1,
            ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
        battery_publisher_ = this->rosnode_->advertise(ops);
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&BatteryStatus::OnUpdate, this));
}

void BatteryStatus::OnUpdate()
{
    common::Time cur_time = this->world->GetSimTime();

    // Get new commands/state
    callback_queue_.callAvailable();

    this->battery_status_msg.header.frame_id = this->model->GetName();
    this->battery_status_msg.header.stamp.sec = cur_time.sec;
    this->battery_status_msg.header.stamp.nsec = cur_time.nsec;

    //this->battery_status_msg.can_timestamp = 1;
    this->battery_status_msg.battery_id = atoi(this->battery_ID_.c_str());           //11-front arm, 12-right arm, 13-back arm, 14-left arm
    this->battery_status_msg.voltage = 12;              // battery voltage (V)
    this->battery_status_msg.current =100;		// battery current (A)
    this->battery_status_msg.percentage = 99;		// battery percentage

    

    battery_publisher_.publish(battery_status_msg);


}
GZ_REGISTER_MODEL_PLUGIN(BatteryStatus);
