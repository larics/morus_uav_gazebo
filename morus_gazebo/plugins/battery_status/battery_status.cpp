#include <morus_gazebo/BatteryStatus.h>

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

    this->pose_topic_ = this->model->GetName() + "/BatteryStatus";
    if (!_sdf->HasElement("poseTopic"))
    {
        ROS_WARN("BatteryStatus plugin (ns = %s) missing <poseTopic>, defaults to \"%s\"", robot_namespace_.c_str(), this->pose_topic_.c_str());
    }
    else this->pose_topic_ = _sdf->GetElement("poseTopic")->Get<std::string>();

    // Ensure that ROS has been initialized
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    // Publisher
    if (!pose_topic_.empty())
    {
        //printf("Stvoril sam publishera!!\n");
        ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<geometry_msgs::PoseStamped>(
            pose_topic_, 1,
            ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
        pose_publisher_ = this->rosnode_->advertise(ops);
    }

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GetPose::OnUpdate, this));
}

void BatteryStatus::OnUpdate()
{
    common::Time cur_time = this->world->GetSimTime();

    // Get new commands/state
    callback_queue_.callAvailable();

    this->model_position.header.frame_id = this->model->GetName();
    this->model_position.header.stamp.sec = cur_time.sec;
    this->model_position.header.stamp.nsec = cur_time.nsec;

    gazebo::math::Pose target_pose = model->GetWorldPose();

    model_position.pose.position.x = target_pose.pos.x;
    model_position.pose.position.y = target_pose.pos.y;
    model_position.pose.position.z = target_pose.pos.z;
    model_position.pose.orientation.x = target_pose.rot.x;
    model_position.pose.orientation.y = target_pose.rot.y;
    model_position.pose.orientation.z = target_pose.rot.z;
    model_position.pose.orientation.w = target_pose.rot.w;

    pose_publisher_.publish(model_position);


}
GZ_REGISTER_MODEL_PLUGIN(BatteryStatus);
