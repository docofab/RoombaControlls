# Roombaのシミュレーション環境のセットアップ(Ubuntu 18.04 LTS + ROS Melodic)

## Ubuntu 18.04 LTSのインストール

1. UbuntuのサイトからUbuntu 18.04 LTSをダウンロードしてインストールする。
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
    mkdir git
    cd git
    git clone https://github.com/karaage0703/jetson-nano-tools.git
    ```

## ROS melodicのインストール

1. 以下のコマンドを入力する。

    ```
    cd ~/git/jetson-nano-tools/
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
    cd ~/git
    git clone https://github.com/docofab/RoombaControlls.git
    cd RoombaControlls/ROS/scripts
    chmod 755 *.sh
    ./install-gazebo-roomba.sh
    ```

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

