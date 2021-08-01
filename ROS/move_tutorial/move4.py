#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

def set_vel(vel_msg, lv, av):
    vel_msg.linear.x  = lv
    vel_msg.linear.y  = 0
    vel_msg.linear.z  = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = av

def main_loop():
    rospy.init_node('move')
    vel_publisher = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)
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
        vel_publisher.publish(vel_msg)
        rospy.loginfo("Velocity: Linear=%s Angular=%s", vel_msg.linear.x, vel_msg.angular.z)
        rate.sleep()
        delta_time = hz * 10.0 / 1000.0
        total_time += delta_time
        rospy.loginfo("total_time=%s delta=%s ", total_time, delta_time)
        if total_time >= move_time:
            if isLinear:
                set_vel(vel_msg, 0, 1.5708 / move_time)
                isLinear = False
            else:
                set_vel(vel_msg, meter / move_time, 0)
                isLinear = True
            total_time = 0.0

if __name__ == '__main__':
        main_loop()
