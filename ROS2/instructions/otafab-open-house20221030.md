# おおたfabオープンハウスデモ

## 準備

- Roomba 側マイコン Raspberry Pi 4 4GB Ubuntu 20.04
- ホストPC ThinkPad X1 VMWare Ubuntu 20.04
- おおたFabのWiFiでは通信ができないので小型ルーターを使用する。PCとRaspberry Piはこれに接続する。
- DualShock4はUSBマイクロ-USB-C変換ケーブルを持っていないので、今回はWindowsPCに接続する。

## デモ１ DualShockリモコンでの操作

```
ros2 run joy_linux joy_linux_node
ros2 topic echo /joy
```

```
ros2 launch teleop_twist_joy teleop-launch.py
ros2 topic echo /cmd_vel
```
- OPTIONSボタン ・・通常モード（/cmd_velの範囲はXは±0.7、Zは±0.4）
- PSボタン・・・・・・ブーストモード（/cmd_velの範囲はXは±1.5、Zは±1.0）

```
ros2 launch create_bringup create_2.launch
```

## デモ２ SLAMとNav2（自律走行）

Raspberry Pi
```
ros2 launch create_bringup create_2l.launch
```

PC
```
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

```
rviz2
```
FileメニューのOpen Configで、ros2_roomba_slam.rvizを指定します。

### SLAM
```
ros2 launch slam_toolbox online_async_launch.py
```

```
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

```
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### Nav2
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

```
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

- 2D Pose Estimateで現在のルンバの位置と向きを設定
- 2D Goal Poseでルンバの目的地と向きを設定します。

### Video

https://gitlab.com/boldhearts/ros2_v4l2_camera  
A ROS 2 camera driver using Video4Linux2 (V4L2).

```
sudo apt install ros-foxy-v4l2-camera
ros2 run v4l2_camera v4l2_camera_node
```

画像データがGUIで確認できます。
```
ros2 run rqt_image_view rqt_image_view
```

Rviz2での確認はAdd→Image→Topic /image_raw

