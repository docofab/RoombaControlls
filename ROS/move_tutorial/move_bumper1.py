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

rospy.init_node('move')
vel_publisher = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)
back = False

def main_loop():
    sub = rospy.Subscriber('/create1/bumper', Bumper, callback)

    vel_msg = Twist()
    set_vel(vel_msg, 0, 0)
    print("Let's move your robot")

    # linear_vel  = input("Input linear velocity [m/s] :")
    # angular_vel = input("Input angular velocity [rad/s] :")
    meter = input("Input meter :")
    # radian = input("Input radian :")
    move_time = 3.0
    total_time = 0.0

    isLinear = True
    linear_vel = meter / move_time
    angular_vel = 0.0

    vel_msg.linear.x  = linear_vel
    vel_msg.angular.z = angular_vel
    hz = 10
    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        if not back:
            vel_publisher.publish(vel_msg)
            rospy.loginfo("Velocity: Linear=%s Angular=%s", vel_msg.linear.x, vel_msg.angular.z)

            delta_time = hz * 10.0 / 1000.0
            total_time += delta_time
            rospy.loginfo("total_time=%s delta=%s ", total_time, delta_time)

            if total_time >= move_time:
            #    if isLinear:
            #        set_vel(vel_msg, 0, 1.5708 / move_time)
            #        isLinear = False
            #    else:
                set_vel(vel_msg, meter / move_time, 0)
                isLinear = True
                total_time = 0.0
        rate.sleep()


def callback(bumper):
    # print bumper
    if bumper.is_left_pressed or bumper.is_right_pressed:
        vel_msg = Twist()
        vel_msg.linear.x = -1.0
        vel_msg.linear.z = 0
        vel_publisher.publish(vel_msg)
        rospy.loginfo("callback Velocity: Linear=%s Angular=%s", vel_msg.linear.x, vel_msg.angular.z)
        back = True
    else:
        back = False


if __name__ == '__main__':
    main_loop()
