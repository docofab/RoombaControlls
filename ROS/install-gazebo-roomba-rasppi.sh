#!/bin/bash -e

# for Raspberry Pi

# Reference sites
# https://demura.net/robot/hard/20405.html

echo "[Update the package]"
sudo apt-get update

echo "[Upgrade the package]"
sudo apt-get upgrade -y

echo "[Installing ROS python and qt4 Packages]"
sudo apt-get install -y python-rosdep python-catkin-tools libqt4-dev

echo "[Making the catkin workspace and testing the catkin build]"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
catkin init
catkin build

echo "[Download the libcreate]"
cd ~/catkin_ws/src
git clone https://github.com/RoboticaUtnFrba/libcreate.git
cd ~/catkin_ws/src/libcreate
git checkout 2.1.0

echo "[Download the create_autonomy]"
cd ~/catkin_ws/src
git clone https://github.com/RoboticaUtnFrba/create_autonomy.git
cd ~/catkin_ws/src/create_autonomy
git checkout a522e080f267253fa2cba237ad32df5132c9aeb5

echo "[Download the RTIMULib]"
cd ~/catkin_ws
./src/create_autonomy/sensors/ca_imu/scripts/install_rtimulib.sh

echo "[Download the etc.]"
cd ~/catkin_ws
sudo apt install -y python3-vcstool
vcs import src < src/create_autonomy/dependencies.repos
rosdep update
rosdep install --from-paths src -yi

echo "[Raspberry Pi 4 Only.]"
rm -rf ~/catkin_ws/src/viso2

echo "[Build the workspace for Raspberry Pi 4]"
cd ~/catkin_ws
catkin build -DCMAKE_BUILD_TYPE=Release -DARM_CROSS_COMPILATION=ON

echo "[Setup environment.]"
cat << EOS >> ~/.bashrc
export GAZEBO_IP=127.0.0.1
export LASER=rplidar
export NUM_ROBOTS=1
export RVIZ=true

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
EOS
source ~/.bashrc