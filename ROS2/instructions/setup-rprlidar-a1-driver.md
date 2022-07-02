# SLAMTEC RPLIDAR A1をROS2 Foxyで動かす手順

すでにワークスペース ~/create_ws ができている前提の手順です。

## ROS2ドライバのインストール

基本的には以下の手順で問題ないはず。

https://github.com/Slamtec/sllidar_ros2

シリアル系の設定は sllidar_ros2/scripts/ で行えそう。

1. 以下の手順でインストールします。
    ```
    $ cd ~/create_ws/src
    $ git clone https://github.com/Slamtec/sllidar_ros2.git
    $ cd ..
    $ colcon build --symlink-install
    ```
    diagnostic-updaterがないというエラーが出たので以下の手順でインストール
    ```
    $ sudo apt-get install ros-foxy-diagnostic-updater
    $ colcon build --symlink-install
    ```

## RPLIDARのデバイス設定

1. 設定ファイルを作成する。
    ```
    $ cd ~/create_ws/src/sllidar_ros2/scripts
    $ sudo bash create_udev_rules.sh
    ```
    colcon_cdがないのでエラーになった。手動で以下のように実行。
    ```
    $ cd ~/create_ws/src/sllidar_ros2/scripts
    $ sudo cp rplidar.rules  /etc/udev/rules.d
    $ sudo service udev reload
    $ sudo service udev restart
    ```
1. USBにRPLIDAR A1を接続し、/dev/rplidarのシンボリックリンクが自動で作成されることを確認します。
    ```
    $ ls -l /dev/rplidar
    ```
## ROS2での動作確認
1. Rplidar A1のノードを動かす。Rvizも起動される。
    ```
    $ ros2 launch sllidar_ros2 view_sllidar_launch.py
    ```
    Rvizが不要であれば以下の手順でノードを動かす。
    ```
    $ ros2 launch sllidar_ros2 sllidar_launch.py
    ```
1. トピックリストで/scanが出ていることを確認する。
    ```
    $ ros2 topic list
    ```