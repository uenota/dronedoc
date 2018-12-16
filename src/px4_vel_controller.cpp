/**
 * @file px4_vel_controller.cpp
 * @brief Node for converting velocity command of move_base to mavros
 * @ author Takaki Ueno
 */

// c++ library
#include <string>
// ROS
#include <ros/ros.h>
// TF
#include <tf/transform_listener.h>
// geometry_msgs
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

//! mavros velocity publisher
ros::Publisher cmd_pub;

//! origin frame name
std::string origin_frame;
//! base frame name of vehicle
std::string vehicle_frame;

//! origin coordinate of origin_frame
tf::Vector3 global_origin(0, 0, 0);
//! pointer to tf listener
tf::TransformListener *listener_ptr;

/**
 * @brief Callback function for velocity command from move_base
 * @param msg Incoming msg
 */
void vel_cmd_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    tf::Vector3 vel_unstamped(msg->linear.x, msg->linear.y, 0);

    tf::StampedTransform vehicle_tf;
    try
    {
        // get tf from origin to base and transform velocity by using the tf
        listener_ptr->lookupTransform(origin_frame, vehicle_frame, ros::Time(0), vehicle_tf);
        vehicle_tf.setOrigin(global_origin);
        vel_unstamped = vehicle_tf * vel_unstamped;
    }
    catch (tf::TransformException& ex)
    {
        // print error and exit function without publishing message
        ROS_ERROR("%s", ex.what());
        return;
    }

    geometry_msgs::TwistStamped vel_cmd;

    // setting header field
    vel_cmd.header.frame_id = origin_frame;
    vel_cmd.header.stamp = ros::Time::now();

    // setting twist field
    vel_cmd.twist.linear.x = vel_unstamped.getX();
    vel_cmd.twist.linear.y = vel_unstamped.getY();
    vel_cmd.twist.linear.z = vel_unstamped.getZ();
    vel_cmd.twist.angular = msg->angular;

    cmd_pub.publish(vel_cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_vel_controller");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    // Fixed frame name
    nh.param<std::string>("origin_frame", origin_frame, "map");
    // Vehicle base name
    nh.param<std::string>("vehicle_frame", vehicle_frame, "base_link");

    // move_base velocity topic
    std::string move_base_topic;
    nh.param<std::string>("move_base_topic", move_base_topic, "/cmd_vel");
    // mavros velocity topic
    std::string mavros_topic;
    nh.param<std::string>("mavros_topic", mavros_topic, "/mavros/setpoint_velocity/cmd_vel");

    cmd_pub = nh.advertise<geometry_msgs::TwistStamped>(mavros_topic, 10);
    // subscriber for velocity command from move base
    ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>(move_base_topic, 10, vel_cmd_cb);

    // listener should initialized after ros::init
    tf::TransformListener listener;

    // use pointer to listener
    // so that listener can be used in callback function
    // and keep listener alive until the node dies
    // Listener starts buffering transform after created
    // and every query will fail if listener becomes out of scope
    listener_ptr = &listener;

    ros::spin();

    return 0;
}