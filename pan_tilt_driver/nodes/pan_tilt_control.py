#!/usr/bin/env python

import math
import rospy

from sensor_msgs.msg import Joy
from pan_tilt_msg.msg import PanTiltCmd

delta_value = 2
pan_tilt_yaw = 0.0
pan_tilt_pitch = 0.0

publisher = 0

def joy_callback(data):
    global publisher
    global pan_tilt_yaw, pan_tilt_pitch

    # button Y
    if data.buttons[0]==1 :
        pan_tilt_pitch -= delta_value
        # rospy.loginfo("up")

    # button B
    if data.buttons[1]==1:
        pan_tilt_yaw -= delta_value
        # rospy.loginfo("right")

    # button A
    if data.buttons[2]==1:
        pan_tilt_pitch += delta_value
        # rospy.loginfo("down")

    # button X
    if data.buttons[3]==1:
        pan_tilt_yaw += delta_value
        # rospy.loginfo("left")

    # button RB
    if data.buttons[5]==1:
        pan_tilt_pitch = 0;
        pan_tilt_yaw = 0;
        # rospy.loginfo("reset")

    if(pan_tilt_pitch > 60):
        pan_tilt_pitch = 60;
    if(pan_tilt_pitch < -60):
        pan_tilt_pitch = -60;

    if(pan_tilt_yaw > 60):
        pan_tilt_yaw = 60;
    if(pan_tilt_yaw < -60):
        pan_tilt_yaw = -60;

    command = PanTiltCmd()
    command.speed = 20.0
    command.yaw = pan_tilt_yaw
    command.pitch = pan_tilt_pitch

    publisher.publish(command)

def main():
    global publisher
    rospy.init_node("pan_tilt_control_node")
    rospy.Subscriber("joy", Joy, joy_callback)
    publisher = rospy.Publisher('/pan_tilt_driver_node/pan_tilt_cmd', PanTiltCmd, queue_size=10)
    
    rospy.loginfo("PanTilt Control Start")
    rospy.spin()

if __name__ == '__main__':
    main()