#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <sensor_msgs/Imu.h>

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
        std::string robot_namespace_, ros_topic_,imu_topic_name_,link_name_;
        //ros::Publisher ros_publisher_;
	sensor_msgs::Imu imuMsg;
	ros::Subscriber imuMsg_subscriber_;
	void onImuMsg(const sensor_msgs::ImuPtr& msg);

        void onUpdate();
    };
}
