#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from ca_msgs.msg import Bumper

def set_vel(vel_msg, lv, av):
    vel_msg.linear.x  = lv
    vel_msg.linear.y  = 0
    vel_msg.linear.z  = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = av

back = False

def main():
    global back

    rospy.init_node('move')
    vel_publisher = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()
    set_vel(vel_msg, 0, 0)
    print("Let's move your robot")

    sub = rospy.Subscriber('/create1/bumper', Bumper, callback)

    hz = 1
    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        vel_publisher.publish(vel_msg)
        rospy.loginfo("Velocity: Linear=%s Angular=%s", vel_msg.linear.x, vel_msg.angular.z)
        if back:
            set_vel(vel_msg, -0.2, -0.5)
        else:
            set_vel(vel_msg, 0.2, 0)

        rate.sleep()


def callback(bumper):
    # print bumper
    global back
    if bumper.is_left_pressed or bumper.is_right_pressed:
        back = True
    else:
        back = False


if __name__ == '__main__':
    main()
