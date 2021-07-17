#!/bin/bash -e

# for Raspberry Pi 4GB

# Reference sites
# https://demura.net/education/18077.html

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
fi

echo "[Update the package]"
sudo apt-get update
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update
sudo apt upgrade -y

echo "[Installing ROS and ROS Packages]"
sudo apt install -y ros-melodic-desktop-full

echo "[Install rosdep and initialize.]"
sudo apt install -y python-rosdep
sudo rosdep init
rosdep update

echo "[Setup bash.]"
cd
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "[Install tools that make installation easier.]"
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
