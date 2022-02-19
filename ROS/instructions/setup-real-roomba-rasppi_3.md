# Roomba実機を動かす環境のセットアップ(Raspberry Pi 3 B+ + Remote PC)

* 参考サイト：https://demura.net/robot/hard/20456.html
* 参考サイト：https://demura.net/education/17957.html


## システム構成

```mermaid
flowchart LR
  subgraph Roomba
    direction TB
    subgraph C2[rplidar]
        direction BT
    end
    subgraph C1[ca_driver]
        direction RL
    end
  end
  subgraph PC
    direction TB
    subgraph B1[roscore]
        direction RL
    end
    subgraph B3[Rviz]
        direction BT
    end
    subgraph B2[Gazebo]
        direction BT
    end
    subgraph B4[keyboard teleop]
        direction BT
    end
  end
  B4 -- /create1/cmd_vel --> B2
  B4 -- /create1/cmd_vel --> C1
  PC <--> WiFi
  Roomba <--> WiFi
  ```


## リモートPCのセットアップ

このPCでroscore, gazebo, Rviz等を動かし、Roombaを制御します。

1. PCにUbuntu 18.04 LTS DesktopとROS Melodicをインストールする  
https://github.com/docofab/RoombaControlls/blob/main/ROS/instructions/setup-gazebo-ubuntu.md
1. IPアドレスを確認する。
    ```
    ip address
    ```
1. 環境変数設定を~/.bashrcに追記する。(PCのIPアドレスを192.168.100.100とした場合)
    ```
    export ROS_MASTER_URI=http://192.168.100.100:11311
    export ROS_HOSTNAME=192.168.100.100
    ```
1. 環境変数を反映する。
    ```
    source ~/.bashrc
    ```

## Raspberry Pi 3のセットアップ

このRaspberry PiにはRoombaのROSドライバノードとLiDARノードを動かします。WiFiネットワークを介してPCのrosmasterに接続します。

### Ubuntu 18.04 LTSのインストール

1. ubuntu-18.04.5-preinstalled-server-armhf+raspi3.img.xzをダウンロード
1. Raspberry Pi imagerでmicroSDカードに書き込む。
1. microSDカード, USB Keyboard, HDMI displayをRaspberry Pi に取り付けて電源を入れる。
1. OSが起動したらubuntu/ubuntuでログインする
1. 初期パスワードの変更メッセージが表示されるのでパスワードを設定する。

### swapの追加

1. メモリが1GBしかないので、swapを設定する。
    ```
    sudo fallocate -l 8G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo vi /etc/fstab
    ```
1. /etc/fstabには以下の内容を記述する。
    ```
    LABEL=writable     /              ext4 defaults 0 0
    LABEL=system-boot  /boot/firmware vfat defaults 0 1
    /swapfile none swap sw 0 0
    ```
1. 設定したらrebootする。

### WiFiの接続

1. WiFiのSSIDとパスワードを設定する。
    ```
    cd /etc/netplan
    sudo vi 50-cloud-init.yaml
    ```

1. 以下のように設定する。WiFi_SSIDとWiFi_Passwordのところは各自の環境に合わせる。
    ```
    network:
        version: 2
        renderer: networkd
        ethernets:
            eth0:
                dhcp4: true
                optional: true
        wifis:
            wlan0:
                dhcp4: true
                optional: true
                access-points:
                    WiFi_SSID:
                    password: WiFi_Password
    ```

1. ネットワークに反映する。

    ```
    sudo netplan apply
    ```
1. pingで疎通確認。ルーターのIPとかにpingしてみる。

    ```
    ping 192.168.xxx.xxx
    ```


### セットアップスクリプトをダウンロード

1. GitHubからクローンする。
    ```
    cd ~
    mkdir git
    cd git
    git clone https://github.com/docofab/RoombaControlls.git
    cd RoombaControlls/ROS
    chmod 755 *.sh
    ```

### ROS melodicのインストール

1. 以下のコマンドを入力する。
    ```
    cd ~/git/RoombaControlls/ROS
    ./install-ros-melodic-rasppi-nogui.sh
    ```

### RoombaのROSドライバのインストール

1. 以下のコマンドを入力する。
    ```
    cd ~/git/RoombaControlls/ROS
    ./install-real-roomba-rasppi_3.sh
    ```

### RPLIDAR ROS パッケージのインストール

1. 新しいRPLIDAR ROS パッケージに置き換えたいので ros-melodic-rplidar-ros がインストールされていたらアンインストールする。

    ```
    sudo apt remove ros-melodic-rplidar-ros
    ```

1. RPLIDAR ROS パッケージ rplidar_rosをクローンしてbuildする。

    ```
    cd ~/catkin_ws/src
    git clone https://github.com/Slamtec/rplidar_ros.git
    catkin build rplidar_ros
    source ~/.bashrc
    ```

### Raspberry Piの準備

1. 電源をバッテリに切り替えるためRaspberry Piをshutdownする。
1. Raspberry Piの電源をモバイルバッテリーに接続し、ディスプレイ、キーボードを接続して起動する。
1. login: が表示されたらubuntuでログインする。
1. WiFi接続で割り当てられたIPアドレスを確認する。
    ```
    ip address
    ```
1. 環境変数設定を~/.bashrcに追記する。(PCのIPアドレスを192.168.100.100、Raspberry PiのIPアドレスを192.168.100.101とした場合)
    ```
    export ROS_MASTER_URI=http://192.168.100.100:11311
    export ROS_HOSTNAME=192.168.100.101
    ```
1. 環境変数を反映する。
    ```
    source ~/.bashrc
    ```

### リモートPCからRaspberry Piにsshログイン

1. リモートPCからRaspberry PiのIPアドレスに対してsshで接続する。
    ```
    ssh ubuntu@192.168.100.101
    ```

### Raspberry PiにシリアルUSBを接続

Roombaは5V, Raspberry PiのGPIOは3.3Vなので、USBシリアル変換を使用する。

1. Raspberry PiとUSBシリアルを接続し、OSでUSBシリアルが認識されているか確認する。
    ```
    ls -l /dev/ttyUSB*
    ```
1. 今回は/dev/ttyUSB0で認識したので、以下のように表示された。  
（実行例）
    ```
    crw-rw---- 1 root dialout 188, 0 May 28 00:28 /dev/ttyUSB0
    ```

### udevの設定

シリアルUSBデバイスをどのユーザからも読み書きできるように設定する。（毎回chmodでの設定が不要になるように）

1. lsusbコマンドを入力する。
    ```
    lsusb
    ```
1. シリアルUSBデバイスのベンダーIDとプロダクトIDをメモする。今回のシリアルUSBはFT232を使用している。  
（実行例）
    ```
    $ lsusb
    Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
    Bus 001 Device 005: ID 046d:c542 Logitech, Inc. 
    Bus 001 Device 004: ID 0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC
    Bus 001 Device 003: ID 3938:1048  
    Bus 001 Device 002: ID 2109:3431 VIA Labs, Inc. Hub
    Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
    ```

1. 以下のコマンドを実行する。
    ```
    sudo vi /etc/udev/rules.d/77-roomba.rules
    ```
1. 以下のように書き込む。
    ```
    KERNEL=="ttyUSB*", ATTRS{idVendor}=="ベンダーID", ATTRS{idProduct}=="プロダクトID", GROUP="dialout", MODE="0666"
    ```
    今回の設定例
    ```
    KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", GROUP="dialout", MODE="0666"
    ```
1. Raspberry PiからUSBシリアルケーブルを一度抜き、再度差し込む。
1. デバイスファイルを確認する。
    ```
    ls -l /dev/ttyUSB*
    ```
1. udevの設定が行われ、パーミッションのotherがrwになっていることを確認する。  
（実行例）
    ```
    $ ls -l /dev/ttyUSB*
    crw-rw-rw- 1 root dialout 188, 0 Jul  3 14:45 /dev/ttyUSB0
    ```

### RoombaとシリアルUSBを接続する

1. RoombaとシリアルUSBを接続する。

    接続は以下の通り
    ```
    FT232(5V)   Roomba
    GND ------- 6,7 GND
    CTS
    5V
    TXD ------- 3 RXD
    RXD ------- 4 TXD
    RTS
    ```
    ルンバのコネクタを上からみた図
    ```
                      (1) (2)
                       =====
    FT232 TXD <----(3) ===== (4)----> FT232 RXD

                    (5) (6) (7)
                         |
                        GND
    ```
1. Roombaの電源を入れる。
1. シリアルUSBのデバイスのパーミッションのotherがrwになっていることを確認する。  
（実行例）
    ```
    $ ls -l /dev/ttyUSB0
    crw-rw-rw- 1 root dialout 188, 0 Jul  3 14:45 /dev/ttyUSB0
    ```
1. 以下のコマンドを入力して、/dev/roomba でアクセスできるようにする。
    ```
    cd /dev
    sudo ln -s ttyUSB0 roomba
    ls -l /dev/roomba 
    ```
1. 以下のように表示されることを確認する。  
（実行例）
    ```
    $ ls -l /dev/roomba
    lrwxrwxrwx 1 root root 7 Jul  3 14:57 /dev/roomba -> ttyUSB0
    ```

### ROS masterの起動

1. リモートPCでターミナルを立ち上げて、ros masterを動かす
    ```
    roscore
    ```
1. 以下のような表示になることを確認する。  
    （実行例）
    ```
    ubuntu@ubuntu:~$ roscore
    ... logging to /home/ocha/.ros/log/88e5c0d6-911b-11ec-b3a4-000c298ba638/roslaunch-ubuntu-4391.log
    Checking log directory for disk usage. This may take a while.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://ubuntu:42671/
    ros_comm version 1.14.12


    SUMMARY
    ========

    PARAMETERS
    * /rosdistro: melodic
    * /rosversion: 1.14.12

    NODES

    auto-starting new master
    process[master]: started with pid [4411]
    ROS_MASTER_URI=http://ubuntu:11311/

    setting /run_id to 88e5c0d6-911b-11ec-b3a4-000c298ba638
    process[rosout-1]: started with pid [4422]
    started core service [/rosout]
    ```

### ROS driver for Roombaの起動

1. リモートPCでターミナルを立ち上げて、sshでRaspberry PiにログインしてRoombaのROSドライバを起動する。
    ```
    ssh ubuntu@192.168.100.63
    roslaunch ca_driver create_2.launch
    ```

1. 以下のような表示になることを確認する。正常に接続できるとRoombaから音が鳴る。  
    （実行例）
    ```
    ubuntu@ubuntu:~$ roslaunch ca_driver create_2.launch
    ... logging to /home/ocha/.ros/log/9446b7ae-dbc3-11eb-a1eb-dca632721712/roslaunch-ubuntu-3710.log
    Checking log directory for disk usage. This may take a while.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://ubuntu:33831/

    SUMMARY
    ========

    PARAMETERS
     * /ca_driver/base_frame: base_footprint
     * /ca_driver/baud: 115200
     * /ca_driver/dev: /dev/roomba
     * /ca_driver/latch_cmd_duration: 0.2
     * /ca_driver/loop_hz: 30.0
     * /ca_driver/namespace: create1
     * /ca_driver/odom_frame: odom
     * /ca_driver/publish_tf: False
     * /ca_driver/robot_model: CREATE_2
     * /ca_driver/tf_prefix: create1
     * /rosdistro: melodic
     * /rosversion: 1.14.11

    NODES
      /
        ca_driver (ca_driver/ca_driver)

    auto-starting new master
    process[master]: started with pid [3720]
    ROS_MASTER_URI=http://localhost:11311

    setting /run_id to 9446b7ae-dbc3-11eb-a1eb-dca632721712
    process[rosout-1]: started with pid [3731]
    started core service [/rosout]
    process[ca_driver-2]: started with pid [3736]
    [ INFO] [1625291866.945971991]: [CREATE] "CREATE_2" selected
    [ INFO] [1625291868.101423483]: [CREATE] Connection established.
    [ INFO] [1625291868.101788872]: [CREATE] Battery level 13.17 %
    [ INFO] [1625291868.274618958]: [CREATE] Ready.
    ```

### rplidarノードの起動

1. リモートPCで新しいターミナルを開いて、Raspberry Piにログインし、RPLiDARのノードを起動する。

    ```
    roslaunch rplidar_ros rplidar.launch
    ```

1. 正常に動けば以下のような表示がでる。
    （実行例）
    ```
    ubuntu@ubuntu:~$ roslaunch rplidar_ros rplidar.launch
    ... logging to /home/ubuntu/.ros/log/2ede11a8-8c9b-11ec-89f9-b827eb039418/roslaunch-ubuntu-3044.log
    Checking log directory for disk usage. This may take a while.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://ubuntu:35783/

    SUMMARY
    ========

    PARAMETERS
     * /rosdistro: melodic
     * /rosversion: 1.14.12
     * /rplidarNode/angle_compensate: True
     * /rplidarNode/frame_id: laser
     * /rplidarNode/inverted: False
     * /rplidarNode/serial_baudrate: 115200
     * /rplidarNode/serial_port: /dev/ttyUSB0

    NODES
      /
        rplidarNode (rplidar_ros/rplidarNode)

    auto-starting new master
    process[master]: started with pid [3055]
    ROS_MASTER_URI=http://localhost:11311

    setting /run_id to 2ede11a8-8c9b-11ec-89f9-b827eb039418
    process[rosout-1]: started with pid [3066]
    started core service [/rosout]
    process[rplidarNode-2]: started with pid [3070]
    [ INFO] [1644735878.487252639]: RPLIDAR running on ROS package rplidar_ros, SDK Version:2.0.0
    [ INFO] [1644735878.547991307]: RPLIDAR S/N: XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    [ INFO] [1644735878.548239482]: Firmware Ver: 1.29
    [ INFO] [1644735878.548415210]: Hardware Rev: 7
    [ INFO] [1644735878.600119489]: RPLidar health status : OK.
    [ERROR] [1644735880.638251192]: Can not start scan: 80008002!
    ```

### キーボード操作ツールの起動

1. リモートPCでターミナルを立ち上げて以下のコマンドを入力する。
    ```
    roslaunch ca_tools keyboard_teleop.launch
    ```
1. 以下のような画面になれば、Gazeboシミュレータと同様にキーボードでRoombaの実機が制御できる状態になる。  
    （実行例）
    ```
    $ roslaunch ca_tools keyboard_teleop.launch
    ... logging to /home/ocha/.ros/log/9446b7ae-dbc3-11eb-a1eb-dca632721712/roslaunch-ubuntu-3779.log
    Checking log directory for disk usage. This may take a while.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://ubuntu:38209/

    SUMMARY
    ========
    
    PARAMETERS
     * /rosdistro: melodic
     * /rosversion: 1.14.11

    NODES
      /create1/
    turtlebot_teleop_keyboard (ca_tools/turtlebot_teleop_key)

    ROS_MASTER_URI=http://localhost:11311

    process[create1/turtlebot_teleop_keyboard-1]: started with pid [3794]

    Control Your Turtlebot!
    ---------------------------
    Moving around:
       u    i    o
       j    k    l
       m    ,    .

    q/z : increase/decrease max speeds by 10%
    w/x : increase/decrease only linear speed by 10%
    e/c : increase/decrease only angular speed by 10%
    space key, k : force stop
    anything else : stop smoothly

    CTRL-C to quit

    currently:	speed 0.2	turn 1
    ```
1. キーボードでRoombaをコントロールする。
