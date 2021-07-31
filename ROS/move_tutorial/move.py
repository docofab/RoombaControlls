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
    linear_vel  = input("Input linear velocity [m/s] :")
    angular_vel = input("Input angular velocity [rad/s] :")
    vel_msg.linear.x  = linear_vel
    vel_msg.angular.z = angular_vel
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vel_publisher.publish(vel_msg)
        rospy.loginfo("Velocity: Linear=%s Angular=%s", vel_msg.linear.x, vel_msg.angular.z)
        rate.sleep()

if __name__ == '__main__':
        main_loop()
