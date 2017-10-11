/// \file  ned_pose_publisher.h
///   \brief Contains ned publsiher and message subscribers for a ROS node using tf.
///

#ifndef NED_POSE_PUBLISHER_H
#define NED_POSE_PUBLISHER_H

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

/// \brief A class that implements ned publisherS.
///
/// Base NED publisher through TF. In addition, this class
/// implements ROS callbacks to subscribe to Odometry sent
/// from Gazebo.
///
class OdometrySubscriber
{
public:

    /// \brief Default constructor.
    ///
    /// Does nothing
    ///
    OdometrySubscriber();

    /// \brief Default destructor.
    ///
    /// Does nothing.
    ///
    ~OdometrySubscriber();

    /// ROS callback function of the measured process value.
    /// \param msg ROS msg containing measured value.
    ///
    void odometryCallback(const nav_msgs::OdometryPtr &msg);
    
    tf::TransformListener tf_listener_;
    nav_msgs::Odometry ned_data_;
private:

};

#endif // PID_CONTROLLER_ROS_H
