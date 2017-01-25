#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <sensor_msgs/Imu.h>
#include <morus_uav_ros_msgs/GmStatus.h>

namespace gazebo
{
    class BaseAttitude2Motion: public ModelPlugin{
    public:
        BaseAttitude2Motion();
        virtual ~BaseAttitude2Motion();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    private:
        physics::ModelPtr model_;
        physics::WorldPtr world_;
	physics::LinkPtr link_;
        event::ConnectionPtr updateConnection;
	
        ros::CallbackQueue callback_queue_;
        ros::NodeHandle* rosnodeHandle_;
        std::string robot_namespace_, ros_topic_,imu_topic_name_,link_name_,connected_ice_motors_;
        
	morus_uav_ros_msgs::GmStatus gm_status_msg_;
	sensor_msgs::Imu imuMsg_;
	ros::Subscriber imuMsg_subscriber_;
	std::vector<ros::Subscriber> gmMsg_subscribers_;
	void onImuMsg(const sensor_msgs::ImuPtr& msg);
	void onICEMsg(const morus_uav_ros_msgs::GmStatusPtr& msg);
        void onUpdate();
    };
}
