#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# 4.正方形を描く
#  １辺がx[m]の正方形の軌跡を描くようにロボットを動かそう。

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
    length  = input("Input length [m] :")
    #time    = input("Input time [s] :")
    #linear_vel  = input("Input linear velocity [m/s] :")
    #angular_vel = input("Input angular velocity [rad/s] :")

    # 90[deg] = pi/4[rad] 
    # angular[rad/s] = pi/4 / time[s]
    # linear[m/s] = length / time[s]

    angular_vel = math.pi / 2.0 / 10.0  # 1/10sec (100count)
    linear_vel  = length / 10.0        # 1/10sec (100count)

    rate = rospy.Rate(10)        # 10Hz=100ms=1/10sec
    isLinear = True
    count = 0

    while not rospy.is_shutdown():
        if isLinear:
            set_vel(vel_msg, linear_vel, 0)
        else:
            set_vel(vel_msg, 0, angular_vel)

        vel_publisher.publish(vel_msg)
        rospy.loginfo("Velocity: Linear=%s Angular=%s", vel_msg.linear.x, vel_msg.angular.z)

        count = count + 1
        if count > 100:
            isLinear = not isLinear
            count = 0

        rate.sleep()



if __name__ == '__main__':
        main_loop()
