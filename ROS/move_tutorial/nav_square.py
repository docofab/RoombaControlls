#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# nav_square.py
# http://dailyrobottechnology.blogspot.com/2014/12/793-navsquarepy.html
# https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_nav/nodes/nav_square.py
#
# 4.正方形を描く
#  １辺がx[m]の正方形の軌跡を描くようにロボットを動かそう。
#

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
import tf
import PyKDL
#from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

class NavSquare():
    def __init__(self):
        # Give the node a name
        rospy.init_node('move', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values? 今回のスクリプトの周期は20Hzです
        rate = 20
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)

        # Set the parameters for the target square
        # 対象となる正方形のパラメータを設定する
        goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
        goal_angle = rospy.get_param("~goal_angle", radians(90))    # degrees converted to radians 度をラジアンに変換
        linear_speed = rospy.get_param("~linear_speed", 0.2)        # meters per second
        angular_speed = rospy.get_param("~angular_speed", 0.7)      # radians per second
        angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians 度をラジアンに変換
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/create1/cmd_vel', Twist)
         
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        # ベースフレームは、TurtleBotではbase_footprintですが、Pi Robotではbase_linkとなります。
        #self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.base_frame = rospy.get_param('~base_footprint', '/base_footprint')
        
        # Find out if the robot uses /base_link or /base_footprint
        #try:
        #    self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
        #    self.base_frame = '/base_footprint'
        #except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        #    try:
        #        self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
        #        self.base_frame = '/base_link'
        #    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        #        rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
        #        rospy.signal_shutdown("tf Exception")
                
        # Initialize the position variable as a Point type
        position = Point()

        # Cycle through the four sides of the square
        # 位置を初期化して、四角形に一辺づつ動くので、繰り返しを4にします。
        for i in range(4):
            # Initialize the movement command
            # 移動のためのTwistメッセージを初期化
            move_cmd = Twist()
            
            # 直進の移動スピードを設定
            # Set the movement command to forward motion
            move_cmd.linear.x = linear_speed
            
            # 現在のオドメトリの値を初期位置・姿勢に代入します。
            # Get the starting position values
            (position, rotation) = self.get_odom()
            
            x_start = position.x
            y_start = position.y
            
            # Keep track of the distance traveled
            distance = 0
            
            # Enter the loop to move along a side
            while distance < goal_distance and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle
                # 速度をパブリッシュして、1サイクルだけスリープします。
                self.cmd_vel.publish(move_cmd)
                rospy.loginfo("Velocity: Linear=%s Angular=%s", move_cmd.linear.x, move_cmd.angular.z)
                
                r.sleep()
        
                # Get the current position
                # 自己位置（オドメトリの値）を取得
                (position, rotation) = self.get_odom()
                
                # Compute the Euclidean distance from the start
                # 初期位置からの移動距離を求めます。
                distance = sqrt(pow((position.x - x_start), 2) +
                                pow((position.y - y_start), 2))

            # Stop the robot before rotating
            # ロボットを止めて90度回転します。
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.loginfo("Velocity: Linear=%s Angular=%s", move_cmd.linear.x, move_cmd.angular.z)
            rospy.sleep(1.0)
            
            # Set the movement command to a rotation
            # 回転速度をパブリッシュ
            move_cmd.angular.z = angular_speed
            
            # Track the last angle measured
            last_angle = rotation
            
            # Track how far we have turned
            turn_angle = 0
            
            # Begin the rotation
            # 回転開始
            while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle
                # 速度をパブリッシュして、1サイクルだけスリープします。
                self.cmd_vel.publish(move_cmd)
                rospy.loginfo("Velocity: Linear=%s Angular=%s", move_cmd.linear.x, move_cmd.angular.z)
                
                r.sleep()
                
                # Get the current rotation
                # 自己位置（オドメトリの値）を取得
                (position, rotation) = self.get_odom()
                
                # Compute the amount of rotation since the last lopp
                # 初期位置からの回転角度を求めます。
                delta_angle = normalize_angle(rotation - last_angle)
                
                turn_angle += delta_angle
                last_angle = rotation

            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            
        # Stop the robot when we are done
        self.cmd_vel.publish(Twist())
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
            
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        NavSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
