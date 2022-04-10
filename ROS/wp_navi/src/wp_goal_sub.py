#!/usr/bin/env python
# -*- coding: utf-8 -*-

# https://www.robotech-note.com/entry/2018/04/16/080816

import rospy
from move_base_msgs.msg import MoveBaseActionGoal

def callback(data):
    pos = data.goal.target_pose.pose
    # 2D Nav Goalで与えた座標(map座標系)、クォータニオン(x,y,z,w)を表示
    print "[({0},{1},0.0),(0.0,0.0,{2},{3})],".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w)

def listener():
    rospy.init_node('goal_sub', anonymous=True)
    rospy.Subscriber("/create1/move_base/goal", MoveBaseActionGoal, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
