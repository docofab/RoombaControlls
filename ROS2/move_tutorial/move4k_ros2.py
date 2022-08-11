#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filename: my_teleop_node.py
import rclpy  # ROS2のPythonモジュールをインポート
import math
from rclpy.node import Node # rclpy.nodeモジュールからNodeクラスをインポート
from std_msgs.msg import String # トピック通信に使うStringメッセージ型をインポート
from geometry_msgs.msg import Twist # トピック通信に使うTwistメッセージ型をインポート


class TeleopPublisher(Node):
    """タイマーのコールバック関数を使い、ロボットを動かす速度指令値のトピックcmd_vel
    をパブリッシュするクラス。
    
    Node: TeleopPublisherクラスが継承するクラス
    """

    def __init__(self):
        """コンストラクタ。パブリッシャーとタイマーを生成する。
        """
        # Nodeクラスのコンストラクタを呼び出し、'teleop_pulisher_node'というノード名をつける。
        super().__init__('teleop_publisher_node') 
        # パブリッシャーの生成。create_publisherの1番目の引数はトピック通信に使うメッセージ型。
        # Twist型は速度指令値を通信するのに使われる。2番目の引数'cmd_vel'はトピック名。
        # 3番目の引数はキューのサイズ。キューサイズはQOS(quality of service)の設定に使われる。
        # サブスクライバーがデータを何らかの理由で受信できないときのキューサイズの上限となる。
        self.publisher = self.create_publisher(Twist,'/cmd_vel', 10)
        
        # タイマーの生成。タイマーのコールバック関数timer_callbackをtimer_period間隔で実行する。
        # この例では0.01[s]秒ごとにtimer_callback関数を呼び出す。
        timer_period = 0.01  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Twistメッセージ型オブジェクトの生成。メンバーにdVector3型の並進速度成分linear、
        # 角速度成分angularを持つ。
        self.vel = Twist()
        print("Let's move your robot")
        length  = input("Input length [m] :")
        self.angular_vel = math.pi / 2.0 / 10.0  # 1/10sec (100count)
        self.linear_vel  = float(length) / 10.0        # 1/10sec (100count)
        self.isLinear = True
        self.count = 0
        
                
    def timer_callback(self):
        """タイマーのコールバック関数。入力キーにより並進及び角速度を増減している。
        input関数はブロックされるので、キーを入力した後にEnterキーを押さなければ速度は変更されない。
        # linear.xは前後方向の並進速度(m/s)。前方向が正。
        # angular.zは回転速度(rad/s)。反時計回りが正。
        """
        if self.isLinear:
            self.vel.linear.x = self.linear_vel
            self.vel.angular.z = 0.0
        else:
            self.vel.linear.x = 0.0
            self.vel.angular.z = self.angular_vel

        self.publisher.publish(self.vel) # 速度指令メッセージをパブリッシュ（送信）する。
        # 端末に並進と角速度を表示する。
        self.get_logger().info("Velocity: Linear=%f angular=%f" % (self.vel.linear.x,self.vel.angular.z)) 

        self.count = self.count + 1
        if self.count > 100:
            self.isLinear = not self.isLinear
            self.count = 0
        
           
def main(args=None):
    rclpy.init(args=args) # rclpyモジュールの初期化
    teleop_publisher = TeleopPublisher() # ノードの作成
    rclpy.spin(teleop_publisher) # コールバック関数が呼び出し
    rclpy.shutdown() # rclpyモジュールの終了処理

if __name__ == '__main__':
    main()