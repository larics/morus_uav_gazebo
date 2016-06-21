#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <morus_uav_ros_msgs/GmStatus.h>
#include <morus_uav_ros_msgs/GmIgnition.h>

#include <math.h>

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
        event::ConnectionPtr updateConnection;
	
        ros::CallbackQueue callback_queue_;
        ros::NodeHandle* rosnode_;
        std::string robot_namespace_, gm_topic_, gm_ID_, gm_ignition_topic_;
        ros::Publisher gm_publisher_;

        morus_uav_ros_msgs::GmStatus gm_status_msg;
	
	ros::Subscriber OnGMIgnitionMsg_subscriber_;
	void OnGMIgnitionMsg(const morus_uav_ros_msgs::GmIgnitionConstPtr& msg);
	int ignition_S, starter_ppm_S;             //setted ignition flag (0-off, 1-on)/setted ppm value for e-starter (range 1000-1900)
	float fuel_level_timeout_;
	float speed_S; //motor speed

        void OnUpdate();
    };
}
