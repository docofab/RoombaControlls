#!/usr/bin/env python
#_*_ coding: utf-8 _*_

# Reference sites
# https://demura.net/robot/hard/20085.html

import rospy
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

_odom_x, _odom_y, _odom_theta = 0.0, 0.0, 0.0

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
    rospy.loginfo("Odometry: x=%s y=%d theta=%s", _odom_x, _odom_y, _odom_theta)

def odometry():
    rospy.init_node('odometry')
    odom_subscriber = rospy.Subscriber('/create1/odom', Odometry, callback_odom)
    rospy.spin()

if __name__ == '__main__':
    odometry()
