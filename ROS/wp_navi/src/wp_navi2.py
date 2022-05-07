#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# HARD2021：ルンバをPythonプログラムでナビゲーションさせよう！
# https://demura.net/robot/hard/20114.html
# 
# wp_goal_navi.pyで生成されたリストを使用するように修正 
#

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi

# クラス
class WpNavi():
    def __init__(self):  # コンストラクタ
        rospy.init_node('wp_navi')  # ノードの初期化
        rospy.on_shutdown(self.shutdown)  # シャットダウン時の処理

        # アクションクライアントの生成
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # アクションサーバーが起動するまで待つ。引数はタイムアウトの時間(秒）
        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        rospy.loginfo("The server comes up")

        self.goal = MoveBaseGoal()  # ゴールの生成
        self.goal.target_pose.header.frame_id = 'map'  # 地図座標系
        self.goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻

        # ここに wp_goal_navi.py で出力されたリストを書き込む
        way_points = [
            [(1.67217516899,-0.108617782593,0.0),(0.0,0.0,-0.706421072541,0.707791825518)],
            [(0.671614408493,-0.68189907074,0.0),(0.0,0.0,0.743972727,0.668209983075)],
            [(0.528070449829,0.025830745697,0.0),(0.0,0.0,0.488727769111,0.872436340199)],
            [(999,999,999),(999,999,999,999)]]

        # メインループ。ウェイポイントを順番に通過
        i = 0
        while not rospy.is_shutdown():
            # ROSではロボットの進行方向がx座標、左方向がy座標、上方向がz座標
            #self.goal.target_pose.pose.position.x = way_point[i][0]
            #self.goal.target_pose.pose.position.y = way_point[i][1]

            self.goal.target_pose.pose.position.x = way_points[i][0][0]
            self.goal.target_pose.pose.position.y = way_points[i][0][1]
            self.goal.target_pose.pose.position.z = way_points[i][0][2]
            self.goal.target_pose.pose.orientation.x = way_points[i][1][0]
            self.goal.target_pose.pose.orientation.y = way_points[i][1][1]
            self.goal.target_pose.pose.orientation.z = way_points[i][1][2]
            self.goal.target_pose.pose.orientation.w = way_points[i][1][3]

            if way_points[i][0][0] == 999:
                break

            rospy.loginfo("Sending goal: No" + str(i + 1))

            self.ac.send_goal(self.goal)
            # サーバーにgoalを送信

            # 結果が返ってくるまで30.0[s] 待つ。ここでブロックされる。
            succeeded = self.ac.wait_for_result(rospy.Duration(30))

            # 結果を見て、成功ならSucceeded、失敗ならFailedと表示
            state = self.ac.get_state()
            if succeeded:
                rospy.loginfo(
                    "Succeeded: No." + str(i + 1) + "(" + str(state) + ")")
            else:
                rospy.loginfo(
                    "Failed: No." + str(i + 1) + "(" + str(state) + ")")

            i = i + 1

    # シャットダウン時の処理
    def shutdown(self):
        rospy.loginfo("The robot was terminated")
        self.ac.cancel_goal()  # ゴールをキャンセル


if __name__ == '__main__':
    # 例外処理。rospy.ROSInterruptExceptionを捕まえる。
    # この例外はCrl+cキーが押されるときに発生するので、
    # この例外処理によりこのノードが終了する。
    try:
        WpNavi()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")
