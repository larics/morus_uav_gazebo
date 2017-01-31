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
        GMStatus();
        virtual ~GMStatus();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    private:
        physics::ModelPtr model;
        physics::WorldPtr world;
	physics::LinkPtr link_;
	physics::JointPtr joint_;
        event::ConnectionPtr updateConnection;
	
        ros::CallbackQueue callback_queue_;
        ros::NodeHandle* rosnode_;
        std::string robot_namespace_, gm_topic_, gm_ID_,link_name_,joint_name_;
        ros::Publisher gm_publisher_;

        morus_uav_ros_msgs::GmStatus gm_status_msg;
	
	double prev_sim_time_;
	double sampling_time_;
	double motor_constant_;
	
	ros::Subscriber OnGMIgnitionMsg_subscriber_;
	void OnGMIgnitionMsg(const morus_uav_ros_msgs::GmIgnitionConstPtr& msg);

        void OnUpdate();
    };
}
