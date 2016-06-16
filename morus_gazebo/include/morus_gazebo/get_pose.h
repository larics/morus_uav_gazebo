#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <geometry_msgs/PoseStamped.h>

namespace gazebo
{
    class GetPose: public ModelPlugin{
    public:
        GetPose();
        virtual ~GetPose();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    private:
        physics::ModelPtr model;
        physics::WorldPtr world;
        event::ConnectionPtr updateConnection;

        ros::CallbackQueue callback_queue_;
        ros::NodeHandle* rosnode_;
        std::string robot_namespace_, pose_topic_;
        ros::Publisher pose_publisher_;

        geometry_msgs::PoseStamped model_position;

        void OnUpdate();
    };
}
