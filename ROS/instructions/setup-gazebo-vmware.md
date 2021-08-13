# Roombaのシミュレーション環境のセットアップ(VMware + Lubuntu 18.04 + ROS Melodic)

## VMware Workstation Playerのインストール

1. VMwareのサイトからVMware Workstation Playerをダウンロードしてインストールする。

## Lubuntu 18.04のインストール

1. Lubuntu-18.04.5-desktop-amd64.isoをダウンロードしてVMware Workstation Playerで新規仮想マシンを作成する。
1. 仮想マシンの作成中にユーザ登録、言語設定、キーボードレイアウトなど聞かれるのでお好みで設定する。
1. 仮想マシンが起動したら登録したユーザでログインする。
1. 以下のように入力する
    ```
    $ sudo apt update
    $ sudo apt upgrade
    $ sudo apt install git
    ```

## ROS Melodicセットアップスクリプトをダウンロード

1. karaageさんのjetson nano用のスクリプトがそのまま使えるので、githubからクローンする。

    ```
    $ cd
    $ mkdir git
    $ cd git
    $ git clone https://github.com/karaage0703/jetson-nano-tools.git
    ```

## ROS melodicのインストール

1. 以下のコマンドを入力する。

    ```
    $ cd ~/git/jetson-nano-tools/
    $ chmod 755 install-ros-melodic.sh
    $ ./install-ros-melodic.sh
    ```

## turtlesimでROS Melodicの動作確認

以下のように操作する。

1. 新たにターミナルを起動して以下のコマンドを入力する。
    ```
    $ roscore
    ```
1. 新たにターミナルを起動して以下のコマンドを入力する。
    ```
    $ rosrun turtlesim turtlesim_node
    ```
1. 新たにターミナルを起動して以下のコマンドを入力する。
    ```
    $ rosrun turtlesim turtle_teleop_key
    ```
1. turtle_teleop_keyを動かしたターミナルの画面上で、キーボードの入力を行うと亀が動くことを確認する。

## Gazeboのインストール

1. 以下のコマンドを入力する。

    ```
    $ cd ~/git
    $ git clone https://github.com/docofab/RoombaControlls.git
    $ cd RoombaControlls/ROS/scripts
    $ chmod 755 *.sh
    $ ./install-gazebo-roomba.sh
    ```

## .bashrcの設定

1. VMware Workstation PlayerでGazeboを動かす場合は以下の設定を.bashrcに追加する。

    ```
    export SVGA_VGPU10=0
    ```

    参考：https://answers.gazebosim.org//question/13214/virtual-machine-not-launching-gazebo/

1. 再度.bashrcを読み込む。

    ```
    $ source ~/.bashrc 
    ```

## Gazeboシュミレーターの起動

1. 新たにターミナルを起動して以下のコマンドを入力する。
    ```
    $ roslaunch ca_gazebo create_empty_world.launch
    ```
1. もう一つターミナルを起動して以下のコマンドを入力する。
    ```
    $ roslaunch ca_tools keyboard_teleop.launch
    ```
1. キーボードでシュミレータのRoombaがコントロールできることを確認する。

