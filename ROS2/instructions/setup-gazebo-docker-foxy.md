Roombaのシミュレーション環境のセットアップ
(DockerのセットアップからROS2 foxyのGazebo環境設置まで)
===============================

## 1. Dockerのセットアップ

### Windows WSL2
@ref:[Windows 10 Home で WSL 2 + Docker を使う][4]

#### ※wslのメモリ制限の設定 
@ref:[WSL2のメモリ割り当て量を変えたい][3]

Powershellで
``` bash
code %USERPROFILE%\.wslconfig
``` 
として、.wslconfigファイルを作成
``` bash
[wsl2]
memory=3GB
processors=2
``` 
を記載する。(記載はプロセッサー数も制限している)

``` bash
wsl --shutdown
``` 

### Linux

dockerのインストール
@ref:[Install Docker Engine on Ubuntu(公式インストールガイド)][2]

sudo の入力を省略する
@ref:[Dockerコマンドをsudoなしで実行する方法][1]
``` bash
# dockerグループがなければ作る
sudo groupadd docker

# 現行ユーザをdockerグループに所属させる
sudo gpasswd -a $USER docker

# dockerデーモンを再起動する
sudo systemctl restart docker

# exitして再ログインすると反映される。
exit
``` 

### Mac

公式サイトの情報に従ってください。

* [Mac に Docker Desktop をインストール](https://docs.docker.jp/docker-for-mac/install.html)

## 2. Docker imageを作る

1. Dockerファイルのリポジトリをクローンし、ファイルの場所に移動

    ```
    git clone https://github.com/sibafb/akiemon_dockerfiles.git
    ```
    
    ```
    cd akiemon_dockerfiles/dockerfiles/foxy
    ```
 
1. ビルドのスクリプトを実行

    ```
    ./build-docker-image.bash
    ```

## 3. 作成したDocker imageでコンテナを動かす

1. Windowsの場合は追加作業にあるWindows用のX Serverを動かしてください。   

1. run のスクリプトを実行

    ```
    ./run-docker-container.bash
    ```
    
    ターミナルが起動するので、ここで操作してください。

1. 終了、再起動

    - ターミナルを閉じれば終了。
    - 以下コマンドで再起動

    ```
    docker start foxy
    ```


## 4. ROS2環境の設定

1. ~/.bashrcに以下のように追加する。
    ```
    source /opt/ros/foxy/setup.bash
    ```
1. 以下のコマンドで確認
    ```
    source ~/.bashrc
    ros2 topic list
    ```
1. これでFoxyの環境ができました。

## 5. Gazeboシミュレーターの起動

1. 以下のパッケージをインストールする
    ```
    sudo apt update
    sudo apt -y install ros-foxy-gazebo-ros-pkgs
    sudo apt -y install ros-foxy-dynamixel-sdk
    sudo apt -y install ros-foxy-turtlebot3-msgs
    sudo apt -y install ros-foxy-turtlebot3
    ```
1. turtlebot3のシミュレーション環境を設定する。
    ```
    mkdir -p ~/colcon_ws/src
    cd ~/colcon_ws/src/
    git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    cd ~/colcon_ws && colcon build --symlink-install
    ```
1. bashrcを設定する。
    ```
    echo "source ~/colcon_ws/install/local_setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

1. 新たにターミナルを起動して以下のコマンドを入力する。
    ```
    $ export TURTLEBOT3_MODEL=burger
    $ ros2 launch turtlebot3_gazebo empty_world.launch.py
    ```
1. もう一つターミナルを起動して以下のコマンドを入力する。
    ```
    $ ros2 run turtlebot3_teleop teleop_keyboard
    ```
1. キーボードでシミュレータのRoombaがコントロールできることを確認する。


## 追加作業：Windows用のX Serverを動かす

1. VcXsrvのインストール  
    - https://sourceforge.net/projects/vcxsrv/

1. XLaunchの起動＆設定

    <img src="https://user-images.githubusercontent.com/36184922/159161402-8cc1045b-2bb9-4d48-b316-6f0465776c51.JPG" width="50%">

    <img src="https://user-images.githubusercontent.com/36184922/159161404-5e6f9910-01b1-4c8c-9e11-57b6fad75499.JPG" width="50%">

    <img src="https://user-images.githubusercontent.com/36184922/159161410-4141f625-dac4-40d0-965b-9aeab1cd4b57.JPG" width="50%">

1. DISPLAY環境変数の設定

    WSLのshellでDISPLAYを設定する。
    ```
    export DISPLAY=`hostname`.mshome.net:0.0
    ```

## トラブルシューティング

1. run-docker-container.bashでエラーになる場合

    GPUがないPCの場合、下記エラーとなることがあります。
    ```
    docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]].  
    ```
    
    その場合は下記の行を削除してください。
    ```run-docker-image.bash
    docker
    run                                                                                                        
    　--
      --gpus all \   ←削除
      --  
    ```
1. WindowsでX Serverと通信できない場合

    Windows firewallの設定を確認してください。

    コントロール パネル→システムとセキュリティ→Windows Defender ファイアウォール→許可されたアプリ

    VcXsrv windows xserver の項目にチェックが入っていない場合はチェックを入れてください。

1. xhost: command not foundと表示される場合  
    例
    ```
    ./run-docker-container.bash: line 11: xhost: command not found
    ```

    x11-xserver-utilsをインストールしてください。

    ```
    sudo apt install x11-xserver-utils
    ```

## 参考サイト
- ### DockerTips
  - [Docker公式インストールガイド][2]
  - [Dockerコマンドをsudoなしで実行する方法][1]
  - [Windows 10 Home で WSL 2 + Docker を使う][4]
  - [WSL2のメモリ割り当て量を変えたい][3]
 
[1]:https://qiita.com/DQNEO/items/da5df074c48b012152ee
[2]:https://docs.docker.com/engine/install/ubuntu/
[3]:https://qiita.com/Ischca/items/121d91eb3b1a0a1cd8a8
[4]:https://qiita.com/KoKeCross/items/a6365af2594a102a817b

- ### ROSのDockerシミュレータ
  - [Docker上でGUIのROS1/ROS2を一瞬でセットアップする方法][5]
  - [ROS/ROS2のGUIをWebブラウザ経由でお手軽に試せるDockerfileを公開しました][6]
  - [Tiryoh氏のgithubリポジトリ][7]

[5]:https://qiita.com/karaage0703/items/957bdc7b4dabfc6639da
[6]:https://memoteki.net/archives/2955
[7]:https://github.com/Tiryoh/docker-ros-desktop-vnc


[8]:https://demura.net/robot/hard/20405.html

ロボットプログラミングⅡ-2021：ROS2演習8 – Happy Miniをプログラムで動かそう！（Python）   
https://demura.net/education/lecture/21781.html

Robotis e-manual turtlebot3   
https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

