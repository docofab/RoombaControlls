#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# nav_square.py
# http://dailyrobottechnology.blogspot.com/2014/12/793-navsquarepy.html
#
# 4.正方形を描く
#  １辺がx[m]の正方形の軌跡を描くようにロボットを動かそう。
#
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

class NavSquare():
    def __init__(self):
        # Give the node a name
        rospy.init_node('nav_square', anonymous=False)
        
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
                
                r.sleep()
                
                # Get the current rotation
                # 自己位置（オドメトリの値）を取得
                (position, rotation) = self.get_odom()
                
                # Compute the amount of rotation since the last lopp
                # 初期位置からの回転角度を求めます。
                delta_angle = normalize_angle(rotation - last_angle)
                
                turn_angle += delta_angle
                last_angle = rotation
