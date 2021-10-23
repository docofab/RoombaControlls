#!/bin/sh
# slam option runs slam_gmapping
export LOCALIZATION=slam
# Open RViz with navigation.rviz
export RVIZ=true
# Specify only 1 robot (optional)
export NUM_ROBOTS=1
# rviz config is navigation
export RVIZ_CONFIG=navigation
# Select the laser that the robot is using
export LASER=rplidar
# Open the simulation in a maze environment
roslaunch ca_gazebo create_house.launch

<< COMMENTOUT
roslaunch ca_gazebo create_autorace.launch
roslaunch ca_gazebo create_playground.launch
roslaunch ca_gazebo create_sweet_house_5.launch
roslaunch ca_gazebo create_cooperative_two_rooms.launch 
roslaunch ca_gazebo create_playpen.launch
roslaunch ca_gazebo create_tea_shop.launch
roslaunch ca_gazebo create_corridor.launch
roslaunch ca_gazebo create_restaurant.launch
roslaunch ca_gazebo create_two_floors_house.launch
roslaunch ca_gazebo create_elevated.launch
roslaunch ca_gazebo create_servicesim.launch
roslaunch ca_gazebo create_warehouse.launch
roslaunch ca_gazebo create_empty_world.launch
roslaunch ca_gazebo create_stage_1.launch
demo.launch
roslaunch ca_gazebo create_house.launch
roslaunch ca_gazebo create_stage_2.launch
include
roslaunch ca_gazebo create_maze.launch
roslaunch ca_gazebo create_sweet_house_3.launch
COMMENTOUT