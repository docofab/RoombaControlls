#!/bin/bash

# Initrize directory
cd ~
mkdir git
mv catkin_ws catkin_ws_save

# Setup catkin_ws
mkdir -p ~/catkin_ws/src
cd catkin_ws
catkin_make
source devel/setup.bash

# Install libfreenect
cd ~/git/
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect/
mkdir build
cd build
cmake ..
sudo make install
cd ../..
sudo cp libfreenect/platform/linux/udev/51-kinect.rules /etc/udev/rules.d
sudo udevadm trigger

# Install rgbd_launch and freenect_stack
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/rgbd_launch.git
git clone https://github.com/ros-drivers/freenect_stack.git
cd ~/catkin_ws
catkin_make
source ~/.bashrc

# Install RTAB-Map standalone library
cd ~/git
git clone https://github.com/introlab/rtabmap.git
cd rtabmap/build
cmake ..
make
sudo make install

# Install RTAB-Map ros package
cd ~/catkin_ws/src
git clone https://github.com/introlab/rtabmap_ros.git
cd ..
catkin_make -j4

# EOF
