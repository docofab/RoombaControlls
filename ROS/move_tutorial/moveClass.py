#!/usr/bin/env python
# -*- coding: utf-8 _*_

## ref HARD2021: ルンバをPythonプログラムで動かそう！ https://demura.net/robot/hard/20101.html
## ハンズオン
# 1.サンプルプログラムの実行
# 上の説明に従ってサンプルプログラムを実行して動作を確認しよう！
# 2.指定した時間だけ移動する
#  指定した時間[s]の間、指定した速度[m/s]と角速度[rad/s]で移動するプログラムを作ろう！
# 3.指定した距離/角度だけ移動する
#  指定した距離[m]/角度[rad]だけ移動するプログラムを作ろう！
# 4.正方形を描く
#  １辺がx[m]の正方形の軌跡を描くようにロボットを動かそう。
# 5.円を描く
#  半径x[m]の円の軌跡を描くようにロボットを動かそう。
# 6.オドメトリ関数の実装
#  ロボットを自作するとオドメトリも自分で実装しなければなりません。それに備えてオドメトリを実装してみましょう。
#  距離[m] = 並進速度[m/s]　×　時間[s]、回転角度[rad] = 角速度[rad/s]　×　時間[s]
#  ロボットの姿勢を考慮して、三角関数を使い進んだ距離と角度を計算する。距離はx軸、y軸に分ける考える。計測時間が必要になるのでネットで調べよう。
#  計算に必要な情報
#  ルンバの車輪間の距離
#  車輪の直径
#  車輪の回転量
#  プログラムができたら、/odomと比較して正しいか確認しよう。

# 4.正方形を描く
#  １辺がx[m]の正方形の軌跡を描くようにロボットを動かそう。

import rospy # rospy http://docs.ros.org/en/jade/api/rospy/html/rospy-module.html
from geometry_msgs.msg import Twist 
import time

class MotionController:
    """
    @class MotionController
    @brief 
    @details 
    @warning 
    @note 
    """
    def __init__(self, rospy_shutdown, vel_publisher, vel_msg, HZ, rate, logger):
        """
        @fn __init__(self, rospy, vel_publisher, vel_msg, HZ, rate, logger)
        @details 
        @param 
        @return None
        @warning 
        @note 
        """
        self.rospy_shutdown = rospy_shutdown
        self.vel_publisher = vel_publisher
        self.vel_msg = vel_msg
        self.HZ = HZ
        self.rate = rate
        self.logger = logger
    # Utility function
    def set_vel(self, lv, av):
        """
        @fn 
        @details 
        @param 
        @return None
        @warning 
        @note 
        """
        self.vel_msg.linear.x = lv 
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = av

        ## Actually Speed limit is needed, but omitted.
    # Basic Move Operation
    def move_stop(self):
        """
        @fn 
        @details 
        @param 
        @return None
        @warning 
        @note 
        """
        self.set_vel(0,0)
        self.vel_publisher.publish(self.vel_msg)
    def move_advance(self,target_distance,maxspeed):
        """
        @fn 
        @details 
        @param 
        @return None
        @warning 
        @note 
        """
        speed = maxspeed
        period = 1.0 / self.HZ
        distance = 0.0
        
        self.set_vel(speed,0)

        while not self.rospy_shutdown:
            distance += speed * period
            self.vel_publisher.publish(self.vel_msg)
            self.logger("Moved distance: distance=%s speed=%s period=%s", distance, speed, period )
            if round(distance, 2) >= round(target_distance, 2):
                break
            self.rate.sleep()

        self.move_stop()
        self.rate.sleep()
    def move_turn(self, target_angle,maxspeed):
        """
        @fn 
        @details 
        @param 
        @return None
        @warning 
        @note 
        """
        speed = 0.2
        period = 1.0 / self.HZ
        angle = 0

        self.set_vel(0,-speed)

        while not self.rospy_shutdown:
            angle += speed * period
            self.vel_publisher.publish(self.vel_msg)
            self.logger("Moved angle: angle=%s ", angle )
            if round(angle, 2) >= round(target_angle, 2):
                self.logger("Break Moved angle: angle=%s target_angle=%s", angle, target_angle )
                break
            self.rate.sleep()

        self.move_stop()
        self.rate.sleep()
    # Sequential Move Operation
    def move_rectangle(self, vertical_length, horizontal_length):
        # Only Counter clockwise
        self.move_advance(vertical_length,1)
        time.sleep(5)
        self.move_turn(1.57, 1)
        time.sleep(5)
        self.move_advance(horizontal_length,1)
        time.sleep(5)
        self.move_turn(1.57, 1)
        time.sleep(5)
        self.move_advance(vertical_length,1)
        time.sleep(5)
        self.move_turn(1.57, 1)
        time.sleep(5)
        self.move_advance(horizontal_length,1)
        time.sleep(5)
        self.move_turn(1.57, 1)
        time.sleep(5)

def main_loop():
    rospy.init_node( 'move' )
    vel_publisher = rospy.Publisher( '/create1/cmd_vel', Twist, queue_size=10 )

    rospy_shutdown = rospy.is_shutdown()
    vel_msg = Twist()
    HZ = 10 # 10Hz for Rate
    rate = rospy.Rate(HZ)
    logger = rospy.loginfo

    motionController1 = MotionController( rospy_shutdown, vel_publisher, vel_msg, HZ, rate, logger )

    print("Let's move your robot")
    print("The Robot moves to draw a rectangle")

    #advance_length = input("Input advance length [m]:")

    vertical_length = input("Input vertical length of rectangle [m]:")
    horizontal_length = input("Input horizontal length of rectangle [m]:")
    
    motionController1.move_rectangle( vertical_length, horizontal_length)

if __name__ == '__main__':
    main_loop()