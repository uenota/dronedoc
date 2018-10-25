#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest

current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

def offboard_node():

    rospy.init_node("offb_node")
    r = rospy.Rate(20)

    rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local",
                                     PoseStamped,
                                     queue_size=1000)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    while not rospy.is_shutdown() and not current_state.connected:
        r.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    for i in range(100):
        local_pos_pub.publish(pose)
        r.sleep()

        if rospy.is_shutdown():
            break

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = "OFFBOARD"

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" \
              and (rospy.Time.now() - last_request > rospy.Duration(5)):

            try:
                offb_set_mode_resp = set_mode_client(offb_set_mode)
                if offb_set_mode_resp.mode_sent:
                    rospy.loginfo("Offboard enabled")
            except rospy.ServiceException as e:
                rospy.logwarn(e)

            last_request = rospy.Time.now()

        else:
          if not current_state.armed \
                and (rospy.Time.now() - last_request > rospy.Duration(5)):

            try:
                arm_cmd_resp = arming_client(arm_cmd)
                if arm_cmd_resp.success:
                    rospy.loginfo("Vehicle armed")
            except rospy.ServiceException as e:
                rospy.logwarn(e)

            last_request = rospy.Time.now()

        local_pos_pub.publish(pose)
        r.sleep()

if __name__ == "__main__":
    try:
        offboard_node()
    except rospy.ROSInterruptException:
        pass