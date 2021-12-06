#!/bin/bash

# Initrize directory
cd ~
mkdir git

# Install libfreenect
cd ~/git
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect/
mkdir build
cd build
cmake ..
sudo make install
cd ../..
sudo cp libfreenect/platform/linux/udev/51-kinect.rules /etc/udev/rules.d
sudo udevadm trigger

# Install RTAB-Map standalone library
cd ~/git
git clone https://github.com/introlab/rtabmap.git
cd rtabmap/build
cmake ..
make
sudo make install

# Setup catkin_build_ws
bash /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_build_ws/src
cd ~/catkin_build_ws
catkin init

# Install rgbd_launch and freenect_stack, RTAB-Map ros package
cd ~/catkin_build_ws/src
git clone https://github.com/ros-drivers/rgbd_launch.git
git clone https://github.com/ros-drivers/freenect_stack.git
git clone https://github.com/introlab/rtabmap_ros.git
cd ~/catkin_build_ws
catkin build

cd ~/catkin_build_ws/
bash devel/setup.bash

# EOF
