#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# 5.円を描く
#  半径x[m]の円の軌跡を描くようにロボットを動かそう。

import rospy
import math
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
    radius  = input("Input radius [m] :")
    time    = input("Input time [s] :")
    #linear_vel  = input("Input linear velocity [m/s] :")
    #angular_vel = input("Input angular velocity [rad/s] :")

    # 360[deg] = 2pi[rad] 
    # angular[rad/s] = 2pi / time[s]
    # linear[m/s] = radius[m] x angular[rad/s]

    # example
    #  deg=360,r=1[m],t=10[sec]
    #  angular = 2pi/10 = 0.628[rad/s]
    #  linear  = 1 x (2pi/10) = 0.628[m/s]
 
    angular_vel = 2 * math.pi / time
    linear_vel  = angular_vel * radius

    set_vel(vel_msg, linear_vel, angular_vel)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        vel_publisher.publish(vel_msg)
        rospy.loginfo("Velocity: Linear=%s Angular=%s", vel_msg.linear.x, vel_msg.angular.z)
        rate.sleep()

if __name__ == '__main__':
        main_loop()
