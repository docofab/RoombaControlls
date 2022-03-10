# Roombaの実機環境でのSLAM

* 参考ページ：[RaspberryPi + ROS で　ルンバを制御 （楽しい ものづくり）](https://goodfield55.blog.fc2.com/blog-entry-16.html)
* 使用ドライバ
    * [GoodField55/libcreate](https://github.com/GoodField55/libcreate/tree/goodfield)
    * [GoodField55/create_autonomy](https://github.com/GoodField55/create_autonomy/tree/goodfield)

## STEP1. Roomba実機環境の準備

テスト環境のIPアドレスは以下のようになっています。
* Ubuntu PC 192.168.0.34
* Raspberry Pi 192.168.0.63

1. Ubuntu PCにログイン
1. Ubuntu PCの環境変数の確認
    ```
    export ROS_MASTER_URI=http://192.168.0.34:11311
    export ROS_HOSTNAME=192.168.0.34
    ```
1. Raspberry Piにモバイルバッテリをつないで起動
1. Ubuntu PCからRaspberry Piにログイン
    ```
    ssh ubuntu@192.168.0.63
    ```
1. Raspberry Piの環境変数の設定
    ```
    export ROS_MASTER_URI=http://192.168.0.34:11311
    export ROS_HOSTNAME=192.168.0.63
    ```

## STEP2. トピックの記録

1. Raspberry Piの設定
    1. ca_driverのconfigファイルを確認  
        ~/catkin_ws/src/create_autonomy/ca_driver/config/default.yaml  
        ※base_frame: "base_footprint"がbase_frame: "base_link" に変更されています。
    1. rplidarのlaunchファイルでlaserとbase_footprintのリンクをpublishする設定を追加する。  
        ~/catkin_ws/src/rplidar_ros/launch/rplidar.launch に以下の行を追加
        ```
        <node pkg="tf" type="static_transform_publisher" name="laser_broadcaster" args="0 0 0.01 0 0 0 base_link laser 100" />
        ```
1. Ubuntu PCでroscoreを起動する。
    ```
    roscore
    ```
1. Ubuntu PCでrosbagで記録する。
    ```
    rosbag record -a
    ```
1. RaspberryPiにsshでログインしルンバのドライバを起動する
    ```
    roslaunch ca_driver create_2.launch
    ```
1. RaspberryPiにsshでログインしRPLiDARのノードを起動する。
    ```
    roslaunch rplidar_ros rplidar.launch
    ```
1. Ubuntu PCでキーボードでルンバを動かして、地図情報を取得する。
    ```
    roslaunch ca_tools keyboard_teleop2.launch
    ```
    ※keyboard_teleop2.launchは、/create1/cmd_velを/cmd_velに変更したもの
1. Ubuntu PCでbugデータを記録しているターミナルでCtrl-Cを入力し記録を停止する。


## STEP3. 地図の保存

1. Ubuntu PCで新しいターミナルを起動して以下を入力する
    ```
    rosrun map_server map_saver -f mymap
    ```
1. カレントディレクトリに mymap.pgm と mymap.yaml というファイル名で地図情報が保存される。
