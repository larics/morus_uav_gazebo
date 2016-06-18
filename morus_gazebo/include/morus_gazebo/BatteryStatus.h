#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <morus_uav_ros_msgs/BatteryStatus.h>

namespace gazebo
{
    class BatteryStatus: public ModelPlugin{
    public:
        BatteryStatus();
        virtual ~BatteryStatus();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    private:
        physics::ModelPtr model;
        physics::WorldPtr world;
        event::ConnectionPtr updateConnection;

        ros::CallbackQueue callback_queue_;
        ros::NodeHandle* rosnode_;
        std::string robot_namespace_, battery_topic_, battery_ID_;
        ros::Publisher battery_publisher_;

        morus_uav_ros_msgs::BatteryStatus battery_status_msg;

        void OnUpdate();
    };
}
