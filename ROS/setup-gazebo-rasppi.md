# Roombaのシミュレーション環境のセットアップ(Raspberry Pi 4)

参考サイト：https://demura.net/education/17957.html

## Ubuntu 18.04 LTSのインストール

1. ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xzをダウンロード
1. Raspberry Pi imagerでmicroSDカードに書き込む。
1. microSDカードをRaspberry Pi に取り付けて電源を入れる。
1. 起動したらubuntu/ubuntuでログイン
1. 以下のように入力する
```
sudo apt update
sudo apt upgrade
sudo apt install xubuntu-desktop
sudo apt install openssh-server
```

## 作業用ユーザアカウントの登録

  * Setting->Users and groups: 任意のユーザを登録。登録時にadministratorに設定すること
  * 登録が終わったら、一度再起動して、作成したユーザでログインする。

## xfceデスクトップ環境の設定

* キーボードレイアウトの設定
  * Settings->Keyboard->Layout: Keyboard layout: Add -> Japanese (OADG 109A)
  * Keyboard model: Sun Type 7 USB(Japanese)/Japanese 106-key
* タイムゾーンの設定
  * Settings->Time and Data Settings: Time zone: Asia/Tokyo
* 日本語化設定
  * Settings -> Language Support: 日本語を追加
  * Keyboard Input method system: fcitxを設定

## swapの追加

SDカードに空きがあればswapを設定する。

```
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo vi /etc/fstab
cat /etc/fstab
LABEL=writable  /        ext4   defaults        0 0
LABEL=system-boot       /boot/firmware  vfat    defaults        0       1
/swapfile none swap sw 0 0
```

設定したらrebootする。

## セットアップスクリプトをダウンロード

GitHubからクローンする。

```
cd ~
mkdir git
cd git
git clone https://github.com/docofab/RoombaControlls.git
cd RoombaControlls/ROS
chmod 755 *.sh
```

## ROS melodicのインストール

```
cd ~/git/RoombaControlls/ROS
./install-ros-melodic-rasppi.sh
```

## Gazeboのインストール

```
cd ~/git/RoombaControlls/ROS
./install-gazebo-roomba-rasppi.sh
```

## シュミレーターの起動

新たにターミナルを起動する
```
roslaunch ca_gazebo create_empty_world.launch
```
もう一つターミナルを起動する。
```
roslaunch ca_tools keyboard_teleop.launch
```
キーボードでシュミレータのRoombaがコントロールできる。

