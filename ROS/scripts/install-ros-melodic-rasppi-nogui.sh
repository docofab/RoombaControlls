#!/bin/bash -e

# for Raspberry Pi

# Reference sites
# https://demura.net/education/18077.html

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
fi

echo "[Set up your keys]"
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "[Installing ROS and ROS Packages]"
sudo apt update
sudo apt install -y ros-melodic-ros-base

echo "[Setup bash.]"
cd
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "[Install tools that make installation easier.]"
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

echo "[Install rosdep and initialize.]"
sudo apt install -y python-rosdep
sudo rosdep init
rosdep update