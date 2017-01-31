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
	// Standard Gazebp variables 
        // Pointer to the MORUS model
        physics::ModelPtr model_;
	// Pointer to the world (coordinates, forces, etc.)
        physics::WorldPtr world_;
	// In essence MORUS base link which receives the calculated forces
	physics::LinkPtr link_;
	// Through updateConnection we achieve binding callbacks for Gazebo events
        event::ConnectionPtr updateConnection;
	// This variable is depricated
        ros::CallbackQueue callback_queue_;
	// Standard ROS stuff
        ros::NodeHandle* rosnodeHandle_;
        std::string robot_namespace_, ros_topic_,imu_topic_name_,link_name_,connected_ice_motors_;
        // Last received gm status msg
	morus_uav_ros_msgs::GmStatus gm_status_msg_;
	// Last received imu msg
	sensor_msgs::Imu imuMsg_;
	// 
	ros::Subscriber imuMsg_subscriber_;
	// Vector holding all the subscribers for the gmMsgs
	std::vector<ros::Subscriber> gmMsg_subscribers_;
	// Vector holding information on received thrust forces
	std::vector<double> gmMsg_thrust_;
	math::Vector3 omega_body_frame_;
	/// This is IMU msg callback - receives IMU status of the UAV
	void onImuMsg(const sensor_msgs::ImuPtr& msg);
	/// Thi is gm status callback - receives GMStatsu msgs (forces, speed, etc..)
	void onIceMsg(const morus_uav_ros_msgs::GmStatusPtr& msg);//,const std::string& topic);
	/// This is Gazebo callback - running on gazebo time to calculate forces from IMU and lump sum of ICE and apply to link
        void onUpdate();
    };
}
