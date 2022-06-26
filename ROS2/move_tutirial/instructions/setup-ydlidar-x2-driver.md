# YDLiDAR X2 をROS2 Foxyで動かす手順

すでにワークスペース ~/create_ws ができている前提の手順です。

## YDLidar-SDKのインストール

1. 以下の手順でYDLidar-SDKをインストールします。ROS2ドライバで必要です。
    ```
    $ cd
    $ git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    $ cd YDLidar-SDK
    $ mkdir build 
    $ cd build 
    $ cmake ..
    $ make 
    $ sudo make install 
    ```
## ROS2ドライバのインストール

1. 以下の手順でインストールします。ydlidra_ros2_driverが最新版です。ydlidar_ros2は古いものなので間違ってcloneしないでください。 （ここではまった）
    ```
    $ cd ~/create_ws/src
    $ git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
    $ cd ~/create_ws
    $ colcon build --symlink-install
    ```
## YDLiDARのデバイス設定

1. 設定ファイルを作成する。
    ```
    $ cd ~/create_ws/src/ydlidar_ros2_driver/startup
    $ sudo bash initenv.sh 
    ```
1. 設定ファイルがインストールされているか確認
    ```
    $ cd /etc/udev/rules.d
    $ ls -l 
    total 80
    -rw-r--r-- 1 root root 63312 Jun 25 02:39 70-snap.snapd.rules
    -rw-r--r-- 1 root root   236 Jun 26 04:34 77-roomba.rules
    -rw-r--r-- 1 root root   122 Jun 26 07:16 ydlidar-2303.rules
    -rw-r--r-- 1 root root   122 Jun 26 07:16 ydlidar-V2.rules
    -rw-r--r-- 1 root root   122 Jun 26 07:16 ydlidar.rules
    ```
1. USBにYDLiDARを接続し、デバイスが認識できるかとアクセス権限を確認します。/dev/ydlidarのシンボリックリンクが自動で作成されます。
    ```
    $ ls -l /dev/ydlidar
    lrwxrwxrwx 1 root root 7 Jun 26 07:17 /dev/ydlidar -> ttyUSB0
    $ ls -l /dev/ttyUSB0
    crw-rw-rw- 1 root dialout 188, 0 Jun 26 07:33 /dev/ttyUSB0
    ```
## 設定ファイルの修正
1. 設定ファイルをLiDARの仕様にあわせて修正します。  
設定ファイルは ~/create_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml にあります。  
（この例はYDLiDAR X2）
    ```
    ydlidar_ros2_driver_node:
      ros__parameters:
        port: /dev/ydlidar
        frame_id: laser_frame
        ignore_array: ""
        baudrate: 115200
        lidar_type: 1
        device_type: 0
        sample_rate: 3
        abnormal_check_count: 4
        resolution_fixed: true
        reversion: false
        inverted: true
        auto_reconnect: true
        isSingleChannel: true
        intensity: false
        support_motor_dtr: true
        angle_max: 180.0
        angle_min: -180.0
        range_max: 8.0
        range_min: 0.01
        frequency: 10.0
        invalid_range_is_inf: false
    ```
## ROS2での動作確認
1. YDLiDARドライバを動かす。
    ```
    $ ros2 launch ydlidar_ros2_driver ydlidar_launch.py 
    ```
1. トピックリストで/scanが出ていることを確認する。
    ```
    $ ros2 topic list 
    ```