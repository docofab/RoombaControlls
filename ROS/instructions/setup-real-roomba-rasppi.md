# Roomba実機を動かす環境のセットアップ(Raspberry Pi 4)

参考サイト：https://demura.net/robot/hard/20456.html

## ROSとcreate_autonomyのインストール

以下を参考にROSとcreate_autonomyをインストールしたRaspberry Pi 4(4GB)環境を準備する。

https://github.com/docofab/RoombaControlls/blob/main/ROS/setup-gazebo-rasppi.md

## Raspberry Piの準備

1. Raspberry Piの電源をモバイルバッテリーに接続し、ディスプレイ、キーボード、マウスを接続して起動する。
1. デスクトップが表示されたらログインする。
1. WiFiに接続し、割り当てられたIPアドレスを確認する。
   ```
   $ ip address
   ```

## Raspberry Piにsshログイン

1. 別のPCからRaspberry PiのIPアドレスに対してsshで接続する。


## Raspberry PiにシリアルUSBを接続

Roombaは5V, Raspberry PiのGPIOは3.3Vなので、USBシリアル変換を使用する。

1. Raspberry Pi 4とUSBシリアルでRoombaを接続すると、OSでUSBシリアルが認識される。今回は/dev/ttyUSB0で認識された。
    ```
    $ ls -l /dev/ttyUSB*
    crw-rw---- 1 root dialout 188, 0 May 28 00:28 /dev/ttyUSB0
    ```

## udevの設定

シリアルUSBデバイスをどのユーザからも読み書きできるように設定する。（毎回chmodでの設定が不要になるように）

1. lsusbコマンドでシリアルUSBデバイスのベンダーIDとプロダクトIDをメモする。今回のシリアルUSBはFT232を使用している。

    ```
    $ lsusb
    Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
    Bus 001 Device 005: ID 046d:c542 Logitech, Inc. 
    Bus 001 Device 004: ID 0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC
    Bus 001 Device 003: ID 3938:1048  
    Bus 001 Device 002: ID 2109:3431 VIA Labs, Inc. Hub
    Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
    ```

1. sudo vi /etc/udev/rules.d/77-roomba.rulesで、以下のように書き込む。
    ```
    KERNEL=="ttyUSB*", ATTRS{idVendor}=="ベンダーID", ATTRS{idProduct}=="プロダクトID", GROUP="dialout", MODE="0666"
    ```
    今回の設定例
    ```
    KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", GROUP="dialout", MODE="0666"
    ```
1. RoombaのUSBシリアルケーブルを一度抜き、再度差し込む。
1. udevの設定が行われ、パーミッションのotherがrwになっていることを確認する。
    ```
    $ ls -l /dev/ttyUSB*
    crw-rw-rw- 1 root dialout 188, 0 Jul  3 14:45 /dev/ttyUSB0
    ```

## RoombaとシリアルUSBを接続する

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
    ```
    $ ls -l /dev/ttyUSB0
    crw-rw-rw- 1 root dialout 188, 0 Jul  3 14:45 /dev/ttyUSB0
    ```
1. 以下のコマンドを入力して、/dev/roomba でアクセスできるようにする。
    ```
    $ cd /dev
    $ sudo ln -s ttyUSB0 roomba
    $ ls -l /dev/roomba 
    lrwxrwxrwx 1 root root 7 Jul  3 14:57 /dev/roomba -> ttyUSB0
    ```

## Roombaを動かす

1. Roombaの電源を入れる
1. 別にターミナルを１つ立ち上げて以下のコマンドを入力する。
    ```
    $  roslaunch ca_driver create_2.launch
    ```
1. 以下のような画面になることを確認する。正常に接続できるとRoombaから音が鳴る。
    ```
    roslaunch ca_driver create_2.launch
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
1. もう一つターミナルを立ち上げて以下のコマンドを入力する。
    ```
    $ roslaunch ca_tools keyboard_teleop.launch
    ```
1. 以下のような画面になれば、Gazeboシュミレータと同様にキーボードでRoombaの実機が制御できる状態になる。
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
1. キーボードでRoombaをコントロールしてみる。
