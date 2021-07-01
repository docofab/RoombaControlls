# Roombaのシミュレーション環境のセットアップ(Raspberry Pi 4)

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

## セットアップスクリプトをダウンロード

```
cd ~
mkdir git
cd git
git clone https://github.com/docofab/RoombaControlls.git
cd RoombaControlls/ROS
chmod 755 *.sh
```

## ROSのインストール

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

```
roslaunch ca_gazebo create_empty_world.launch
roslaunch ca_tools keyboard_teleop.launch
```
