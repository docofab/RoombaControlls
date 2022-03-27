# Roombaの実機環境でのgmapping-SLAM

* 使用ドライバ
    * https://github.com/kanpapa/create_autonomy/tree/kanpapa-patch-1
* テスト環境のIPアドレスは以下のようになっています。
    * Ubuntu PC 192.168.0.34
    * Raspberry Pi 192.168.0.63

## 構成図

```mermaid
flowchart TB
  subgraph Roomba
    direction TB
    subgraph C1[roscore]
        direction TB
    end
    subgraph C2[rplidar]
        direction TB
    end
    subgraph C3[ca_driver]
        direction TB
    end
  end
  subgraph PC
    direction TB
    subgraph B3[keyboard teleop]
        direction TB
    end
    subgraph B6[Rviz]
        direction TB
    end
    subgraph B4[rosbag]
        direction TB
    end
    subgraph B5[bugfile]
        direction TB
    end
    subgraph B7[gmapping]
        direction TB
    end
    subgraph B8[map]
        direction TB
    end
  end
B3 -- /create1/cmd_vel --> C3
C2 -- /create1/rplidar/scan --> B4
C2 -- /tf --> B4
C3 -- /create1/rplidar/scan --> B7
C3 -- /tf --> B7
B4 --> B5
B7 --> B8
Roomba <--> WiFi
PC <--> WiFi
```

## STEP1. Roomba実機環境(Raspberry Pi)の準備

1. Raspberry Piにモバイルバッテリをつないで起動する。
1. Raspberry PiのUSBにRoombaとLiDARを接続する。
1. Ubuntu PCにログインし、以下のコマンドでRaspberry PiのIPを探す。
    ```
    nmap -sP 192.168.100.0/24
    ```
1. Ubuntu PCからRaspberry Piにログイン
    ```
    ssh ubuntu@192.168.0.63
    ```
1. Raspberry Piの~/.bashrcに環境変数の設定をする。
    ```
    export ROS_MASTER_URI=http://192.168.0.63:11311
    export ROS_HOSTNAME=192.168.0.63
    ```
1. 環境変数を設定する。
    ```
    source ~/.bashrc
    ```
1. ca_driverのconfigファイルを確認する    
    ~/catkin_ws/src/create_autonomy/ca_driver/config/default.yamlのpublish_tf: がtrueであることを確認する。
    ```
    # Whether to publish the transform between odom_frame and base_frame
    #publish_tf: false
    publish_tf: true
    ```
    ※この設定がfalseだと、create1/odomとcreate1/base_footprintをつなぐtfが配信されない。
1. ca_bringupを起動する。
    ```
    roslaunch ca_bringup minimal2.launch
    ```

## STEP2. Ubuntu PCとRoombaの接続設定

1. Ubuntu PCの~/.bashrcに環境変数の設定をする。ROS_MASTERはRaspberry Piになる。
    ```
    export ROS_MASTER_URI=http://192.168.0.63:11311
    export ROS_HOSTNAME=192.168.0.34
    ```
1. 環境変数を設定する。
    ```
    source ~/.bashrc
    ```
1. ROSコマンドを入力してRoombaのトピックが見えるか確認する。
    ```
    rostopic list
    ```

## STEP3. トピックの記録

1. Ubuntu PCでrosbagで記録する。
    ```
    rosbag record -a
    ```
1. Ubuntu PCでキーボードでルンバを動かして、地図情報を取得する。
    ```
    roslaunch ca_tools keyboard_teleop.launch
    ```
1. Ubuntu PCでbugデータを記録しているターミナルでCtrl-Cを入力し記録を停止する。

## STEP4. gmapping-SLAMで地図をつくる

### 方法１：rosbagファイルから作成する

1. gmappingを起動する
    ```
    roslaunch ca_slam slam_gmapping2.launch
    ```
1. Rvizを起動する。
    ```
    roslaunch ca_tools rviz.launch
    ```
1. トピックを再生する
    ```
    rosbag play --clock 2022-03-19-10-44-20.bag
    ```
1. トピックの再生が終了したら、Ubuntu PCで新しいターミナルを起動して以下を入力する
    ```
    rosrun map_server map_saver -f mymap
    ```
1. カレントディレクトリに mymap.pgm と mymap.yaml というファイル名で地図情報が保存される。
1. 地図が取得できたら、gmappingをctrl-cで停止する。

### 方法２：リアルタイムで作成する

STEP3のrosbagを保存せずにリアルタイムで作成することもできる。ただし正常に動作しない場合のデバックがやりにくい点に注意。

1. gmappingを起動する
    ```
    roslaunch ca_slam slam_gmapping2.launch
    ```
1. Rvizを起動する。
    ```
    roslaunch ca_tools rviz.launch
    ```
1. Ubuntu PCでキーボードでルンバを動かして、地図情報を取得する。
    ```
    roslaunch ca_tools keyboard_teleop.launch
    ```
1. リアルタイムで地図ができあがったら、Ubuntu PCで新しいターミナルを起動して以下を入力する
    ```
    rosrun map_server map_saver -f mymap
    ```
1. カレントディレクトリに mymap.pgm と mymap.yaml というファイル名で地図情報が保存される。
1. 地図が取得できたら、gmappingをctrl-cで停止する。

## 課題
* Roombaのodom情報があまり正確でない。特に回転方向がずれるようなので、なるべく直進で取得すると良い。
