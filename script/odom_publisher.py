#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
import tf

# Callback for local position
local_pos = PoseStamped()
def local_pos_cb(msg):
    # Need global declaration in order to assign global variable
    global local_pos
    local_pos = msg

# Callback for local velocity
local_vel = TwistStamped()
def local_vel_cb(msg):
    # Need global declaration in order to assign global variable
    global local_vel
    local_vel = msg

def odom_publisher():
    rospy.init_node("odom_publisher")

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

    local_pos_sub = rospy.Subscriber("mavros/local_position/pose",
                                      PoseStamped,
                                      callback=local_pos_cb)
    local_vel_sub = rospy.Subscriber("mavros/local_position/velocity",
                                      TwistStamped,
                                      callback=local_vel_cb)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        pose = local_pos.pose
        twist = local_vel.twist

        # Create quaternion from euler angle of local position
        odom_quat = tf.transformations.quaternion_from_euler(pose.orientation.x,
                                                             pose.orientation.y,
                                                             pose.orientation.z)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        # Global frame is "odom"
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = pose.position.x
        odom.pose.pose.position.y = pose.position.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = odom_quat

        # Set robot's base frame as odometry child frame
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = twist.linear.x
        odom.twist.twist.linear.y = twist.linear.y
        odom.twist.twist.angular.z = twist.angular.z

        odom_pub.publish(odom)
        rate.sleep()

# Only run if executed as top level script
if __name__ == "__main__":
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass