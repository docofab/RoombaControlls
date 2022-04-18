# Roombaのシミュレーション環境のセットアップ(Ubuntu 18.04 LTS + ROS Melodic)

## Ubuntu 18.04 LTSのインストール

1. UbuntuのサイトからUbuntu 18.04 LTSをダウンロードしてインストールします。Ubuntu Desktop 日本語 Remixの利用をお勧めします。  
https://www.ubuntulinux.jp/download/ja-remix
1. Ubuntuが起動したら登録したユーザでログインする。
1. 以下のように入力する
    ```
    sudo apt update
    sudo apt upgrade
    sudo apt install git
    ```

## ROS Melodicセットアップスクリプトをダウンロード

1. karaageさんのjetson nano用のスクリプトがそのまま使えるので、githubからクローンする。

    ```
    cd
    git clone https://github.com/karaage0703/jetson-nano-tools.git
    ```

## ROS melodicのインストール

1. 以下のコマンドを入力する。

    ```
    cd ~/jetson-nano-tools/
    chmod 755 install-ros-melodic.sh
    ./install-ros-melodic.sh
    ```

## turtlesimでROS Melodicの動作確認

以下のように操作する。

1. 新たにターミナルを起動して以下のコマンドを入力する。
    ```
    roscore
    ```
1. 新たにターミナルを起動して以下のコマンドを入力する。
    ```
    rosrun turtlesim turtlesim_node
    ```
1. 新たにターミナルを起動して以下のコマンドを入力する。
    ```
    rosrun turtlesim turtle_teleop_key
    ```
1. turtle_teleop_keyを動かしたターミナルの画面上で、キーボードの入力を行うと亀が動くことを確認する。

## Gazeboのインストール

1. 以下のコマンドを入力する。

    ```
    cd ~
    git clone https://github.com/docofab/RoombaControlls.git
    cd RoombaControlls/ROS/scripts
    chmod 755 *.sh
    ./install-gazebo-roomba.sh
    ```

## 環境変数の追加設定（VMware Workstation Playerを利用する場合のみ）

1. VMware Workstation PlayerでUbuntuを動かしている場合は以下のコマンドを入力して環境変数を設定する。これを行わないとGazeboが動きません。

    ```
    echo "export SVGA_VGPU10=0" >> ~/.bashrc
    source ~/.bashrc
    ```

    参考：https://answers.gazebosim.org//question/13214/virtual-machine-not-launching-gazebo/


## Gazeboシミュレーターの起動

1. 新たにターミナルを起動して以下のコマンドを入力する。
    ```
    roslaunch ca_gazebo create_empty_world.launch
    ```
1. もう一つターミナルを起動して以下のコマンドを入力する。
    ```
    roslaunch ca_tools keyboard_teleop.launch
    ```
1. キーボードでシミュレータのRoombaがコントロールできることを確認する。

