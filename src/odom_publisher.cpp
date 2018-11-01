#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  local_pos = *msg;
}

geometry_msgs::TwistStamped local_vel;
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  local_vel = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_publisher");

  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 50, local_pos_cb);
  ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 50, local_vel_cb);

  ros::Rate rate(20);

  while(ros::ok())
  {
    ros::spinOnce();

    tf::Quaternion tf_quat = tf::createQuaternionFromYaw(local_pos.pose.orientation.z);

    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(tf_quat, odom_quat);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = local_pos.pose.position.x;
    odom.pose.pose.position.y = local_pos.pose.position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = local_vel.twist.linear.x;
    odom.twist.twist.linear.y = local_vel.twist.linear.y;
    odom.twist.twist.angular.z = local_vel.twist.angular.z;

    odom_pub.publish(odom);
    rate.sleep();
  }
}