#include <ned_pose_publisher/ned_pose_publisher.h>

OdometrySubscriber::OdometrySubscriber()
{
    
}

OdometrySubscriber::~OdometrySubscriber()
{
  
}

void OdometrySubscriber::odometryCallback(const nav_msgs::OdometryPtr &msg)
{
  tf::Stamped<tf::Pose> enuPose(tf::Pose(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w), 
					 tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)), 
					  msg->header.stamp,"world");
  tf::Stamped<tf::Pose> nedPose;
  
  tf_listener_.transformPose("world_ned",enuPose,nedPose);
  geometry_msgs::PoseStamped nedPoseMsg;
  tf::poseStampedTFToMsg(nedPose,nedPoseMsg);
  
  
  
  tf::Stamped<tf::Point> enuTwist( tf::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z), 
					  msg->header.stamp,"world");
  tf::Stamped<tf::Point> nedTwist;
  tf_listener_.transformPoint("world_ned",enuTwist,nedTwist);
  geometry_msgs::PointStamped nedTwistMsg;
  tf::pointStampedTFToMsg(nedTwist,nedTwistMsg);
  //data_=*msg;
  
  ned_data_ = *msg;
  ned_data_.pose.pose.position = nedPoseMsg.pose.position;
  ned_data_.twist.twist.linear.x = nedTwistMsg.point.x;
  ned_data_.twist.twist.linear.y = nedTwistMsg.point.y;
  ned_data_.twist.twist.linear.z = nedTwistMsg.point.z;
  ned_data_.header.frame_id = "world_ned";
  ned_data_.child_frame_id = "local";
}



int main(int argc, char** argv){
  ros::init(argc, argv, "ned_odom_publisher");

  ros::NodeHandle node;

  ros::Publisher morus_odometry = 
    node.advertise<nav_msgs::Odometry>("ned/odometry", 10);
  
  OdometrySubscriber *odom_data = new OdometrySubscriber();
  
  ros::Subscriber ref_sub = node.subscribe("odometry", 1, &OdometrySubscriber::odometryCallback, odom_data);

  //tf::TransformListener listener;
  unsigned int seq = 0;
  ros::Rate rate(10.0);
  
  while (node.ok()){
    ros::spinOnce();
    /*tf::StampedTransform transform;
    try{
      listener.lookupTransform("Observed from here", "Observing this",  
                               ros::Time(0), transform);
      listener.lookupTransform("world_ned", "local",  
                               ros::Time(0), transform);
      
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = "world";
      pose_msg.header.seq = seq++;
      pose_msg.header.stamp = ros::Time::now();
      
      pose_msg.pose.position.x = transform.getOrigin().x();
      pose_msg.pose.position.y = transform.getOrigin().y();
      pose_msg.pose.position = odom_data->data_.pose.pose.position;
      odom_data->data_.pose.pose.position = pose_msg.pose.position;
      geometry_msgs::PoseStamped pose_msg_ned;
      listener.transformPoint("world_ned", &pose_msg, &pose_msg_ned);
      odom_data->data_.pose.pose.position = pose_msg_ned.pose.position;
      
      
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }*/
    
    morus_odometry.publish(odom_data->ned_data_);
    rate.sleep();
  }
  return 0;
};