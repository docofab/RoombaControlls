#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from math import pi


# クラス
class WpNavi():
    def __init__(self):  # コンストラクタ
        rospy.init_node('wp_navi')  # ノードの初期化
        rospy.on_shutdown(self.shutdown)  # シャットダウン時の処理

        rospy.Subscriber('button_pushed', Bool, self.button_pushed)

        # アクションクライアントの生成
        self.ac = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
        # アクションサーバーが起動するまで待つ。引数はタイムアウトの時間(秒）
        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        rospy.loginfo("The servers comes up")

        self.goal = MoveBaseGoal()  # ゴールの生成
        self.goal.target_pose.header.frame_id = 'map'  # 地図座標系
        self.goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻

        # [x,y,theta]
        self.way_point_robot1 = [[ 6.1, 0.8, 0.0 * pi], [6.1, 4.2, 0.5 * pi],
                     [ 2.7, 4.2, -1.0 * pi], [2.7, 0.8, -0.5 * pi],
                     [999, 999, 999]]

        self.i = 0

    def button_pushed(self, button_pushed):
        if button_pushed.data == True:
            # ROSではロボットの進行方向がx座標、左方向がy座標、上方向がz座標
            self.goal.target_pose.pose.position.x = self.way_point_robot1[self.i][0]
            self.goal.target_pose.pose.position.y = self.way_point_robot1[self.i][1]
            if self.way_point_robot1[self.i][0] == 999:
                return

            q = tf.transformations.quaternion_from_euler(0, 0, self.way_point_robot1[self.i][2])
            self.goal.target_pose.pose.orientation = Quaternion(
                q[0], q[1], q[2], q[3])

            rospy.loginfo("Sending goal: No" + str(self.i + 1))

            self.ac.send_goal(self.goal)
            # サーバーにgoalを送信

            # 結果が返ってくるまで30.0[s] 待つ。ここでブロックされる。
            succeeded = self.ac.wait_for_result(rospy.Duration(30))

            # 結果を見て、成功ならSucceeded、失敗ならFailedと表示
            state = self.ac.get_state()
            if succeeded:
                rospy.loginfo(
                    "Succeeded: No." + str(self.i + 1) + "(" + str(state) + ")")
            else:
                rospy.loginfo(
                    "Failed: No." + str(self.i + 1) + "(" + str(state) + ")")

            self.i = self.i + 1
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
        while not rospy.is_shutdown():
            rospy.spin()
            delay(5)
    except rospy.ROSInterruptException:
        rospy.loginfo("WP navigation finished.")