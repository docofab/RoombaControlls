Roombaのシミュレーション環境のセットアップ
(DockerのセットアップからROSのGazebo環境設置まで)
===============================

## 1.Docker を入れる

### Windows
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

### LinuxでのTIPS

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

## 2. 既存イメージをpullする
@ref:[Docker上でGUIのROS1/ROS2を一瞬でセットアップする方法][5],
[ROS/ROS2のGUIをWebブラウザ経由でお手軽に試せるDockerfileを公開しました][6]

### docker pullによる方法
``` bash
docker pull tiryoh/ros-desktop-vnc
``` 

``` bash
docker run -p 6080:80 --shm-size=512m tiryoh/ros-desktop-vnc:melodic
```



### docker composeによる方法
``` bash

``` 

### アクセス確認

(デフォルトなら)ブラウザで[http://127.0.0.1:6080](http://127.0.0.1:6080) にアクセス

## 3. Roombaの操作ライブラリをインストールする。

## 4. ビルドを通す

## 5. 完成


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

- ### CreateAutonomy
  - [HARD2021: Gazeboシミュレータでルンバを動かそう！][7]

[7]:https://demura.net/robot/hard/20405.html

