/// This class inherits standard Gazebo plugins and is used to publish 
/// GM Status msgs

#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <morus_uav_ros_msgs/GmStatus.h>
#include <morus_uav_ros_msgs/GmIgnition.h>

namespace gazebo
{
    class GMStatus: public ModelPlugin{
    public:
	/// Class constructor
        GMStatus();
        virtual ~GMStatus();
	/// Parent function called through Gazebo to gather information about the model from
	/// the provided sdf file.
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    private:
	/// Standard Gazebo variables pointing to model information
        physics::ModelPtr model;
        physics::WorldPtr world;
	/// Link is not used at the moment TO DO: Check why one cannot collect forces
	physics::LinkPtr link_;
	physics::JointPtr joint_;
        event::ConnectionPtr updateConnection;
	/// Callback_queue deprecated
        ros::CallbackQueue callback_queue_;
	/// Standard ROS parts
        ros::NodeHandle* rosnode_;
	/// Corresponding string variables used to grab correct parameters from sdf model
        std::string robot_namespace_, gm_topic_, gm_ID_,link_name_,joint_name_;
	/// ROS publisher
        ros::Publisher gm_publisher_;
        morus_uav_ros_msgs::GmStatus gm_status_msg;
	
	/// Variables used to calculate DT
	double prev_sim_time_;
	double sampling_time_;
	/// Motor constant from rotor_sim used to calculate the thrust of the motor
	double motor_constant_;
	/// ROS subscriber for ignition msgs
	ros::Subscriber OnGMIgnitionMsg_subscriber_;
	void OnGMIgnitionMsg(const morus_uav_ros_msgs::GmIgnitionConstPtr& msg);
	/// This is Gazebo callback 
        void OnUpdate();
    };
}
