#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped

class PX4VelocityController:

    def __init__(self):
        self.origin_frame = rospy.get_param("origin_frame",
                                            "map")
        self.vehicle_frame = rospy.get_param("vehicle_frame",
                                             "base_link")
        self.move_base_topic = rospy.get_param("move_base_topic",
                                               "/cmd_vel")
        self.mavros_topic = rospy.get_param("mavros_topic",
                                            "/mavros/setpoint_velocity/cmd_vel")

        self.transformer = tf.TransformListener()
        self.cmd_pub = rospy.Publisher(self.mavros_topic,
                                       TwistStamped,
                                       queue_size=10)
        self.cmd_sub = rospy.Subscriber(self.move_base_topic,
                                        Twist, self.vel_cmd_cb)

    def vel_cmd_cb(self, msg):
        linear_vel = Vector3Stamped()
        linear_vel.header.frame_id = self.vehicle_frame
        linear_vel.header.stamp = rospy.Time().now()

        linear_vel.vector.x = msg.linear.x
        linear_vel.vector.y = msg.linear.y
        linear_vel.vector.z = 0

        try:
            self.transformer.waitForTransform(self.vehicle_frame, self.origin_frame, rospy.Time().now(), rospy.Duration(0.05))
            linear_vel = self.transformer.transformVector3(self.origin_frame, linear_vel)
        except tf.Exception as e:
            rospy.logerr(e)
            return

        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = rospy.Time().now()
        vel_cmd.header.frame_id = self.origin_frame

        vel_cmd.twist.linear.x = linear_vel.vector.x
        vel_cmd.twist.linear.y = linear_vel.vector.y
        vel_cmd.twist.linear.z = linear_vel.vector.z
        vel_cmd.twist.angular.z = msg.angular.z

        self.cmd_pub.publish(vel_cmd)


if __name__ == "__main__":
    rospy.init_node("px4_vel_controller")
    rate = rospy.Rate(20)
    PX4VelocityController()
    rospy.spin()