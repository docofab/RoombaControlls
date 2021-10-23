#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# 4.正方形を描く
#  １辺がx[m]の正方形の軌跡を描くようにロボットを動かそう。
#
# Reference sites
# https://demura.net/robot/hard/20101.html
# https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_nav/nodes/nav_square.py

""" nav_square.py - Version 0.1 2012-03-24
    A basic demo of the using odometry data to move the robot
    along a square trajectory.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry   
import tf
from tf.transformations import euler_from_quaternion
from math import radians, copysign, sqrt, pow, pi

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0

def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

def callback_odom(msg):
    global _odom_x, _odom_y, _odom_theta 
    _odom_x = msg.pose.pose.position.x  
    _odom_y = msg.pose.pose.position.y  
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q = (qx, qy, qz, qw)
    e = euler_from_quaternion(q)
    _odom_theta = e[2] 
    #rospy.loginfo("Odomery: x=%s y=%s theta=%s", _odom_x, _odom_y, _odom_theta)

def main_loop():
    rospy.init_node('move')
    cmd_vel = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)
    odom_subscriber = rospy.Subscriber('/create1/odom', Odometry, callback_odom)

    print("Let's move your robot")

    rate = rospy.Rate(20)        # 20Hz

    # Set the parameters for the target square
    goal_distance = 1.0      # meters
    goal_angle = radians(90) # degrees converted to radians
    linear_speed = 0.2       # meters per second
    angular_speed = 0.7      # radians per second
    angular_tolerance = radians(2) # degrees to radians

    # Initialize the position variable as a Point type
    position = Point()

    # Cycle through the four sides of the square
    for i in range(4):
        # Initialize the movement command
        move_cmd = Twist()
        
        # Set the movement command to forward motion
        move_cmd.linear.x = linear_speed
        
        # Get the starting position values     
        #(position, rotation) = get_odom()
        position.x = _odom_x
        position.y = _odom_y
    	rotation = _odom_theta

        x_start = position.x
        y_start = position.y

        # Keep track of the distance traveled
        distance = 0

        # Enter the loop to move along a side
        while distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            cmd_vel.publish(move_cmd)
            
            rate.sleep()

            # Get the current position
            #(position, rotation) = get_odom()
            position.x = _odom_x
            position.y = _odom_y
    	    rotation = _odom_theta
            
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - x_start), 2) + 
                            pow((position.y - y_start), 2))
            
        # Stop the robot before rotating
        move_cmd = Twist()
        cmd_vel.publish(move_cmd)
        rospy.sleep(1.0)
        
        # Set the movement command to a rotation
        move_cmd.angular.z = angular_speed
        
        # Track the last angle measured
        last_angle = rotation
        
        # Track how far we have turned
        turn_angle = 0
        
        # Begin the rotation
        while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            cmd_vel.publish(move_cmd)
            
            rate.sleep()
            
            # Get the current rotation
            #(position, rotation) = get_odom()
            position.x = _odom_x
            position.y = _odom_y
    	    rotation = _odom_theta
            
            # Compute the amount of rotation since the last lopp
            delta_angle = normalize_angle(rotation - last_angle)
            
            turn_angle += delta_angle
            last_angle = rotation

        move_cmd = Twist()
        cmd_vel.publish(move_cmd)
        rospy.sleep(1.0)
    
    # Stop the robot when we are done
    cmd_vel.publish(Twist())

if __name__ == '__main__':
        main_loop()
