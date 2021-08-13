# よくつかうコマンド一覧

## roscoreの起動

```
$ roscore
```

## turtlesimの起動

```
$ rosrun turtlesim turtlesim_node
```

## turtlesimをキーボードで動かす

```
$ rosrun turtlesim turtle_teleop_key
```

## gazeboの起動

```
$ export RVIZ=false     (RVIZを使わない場合)
$ roslaunch ca_gazebo create_empty_world.launch
```

## GazeboのRoombaをキーボードで動かす場合

```
$ roslaunch ca_tools keyboard_teleop.launch
```

## Roomba実機を動かす場合

```
$ roslaunch ca_driver create_2.launch
```

## Roombaに速度指令のトピックを配信する。

例
* x軸方向に0.2m/s
* z軸まわりに0rad/s
* オプション-rは配信する周期Hz(デフォルトは10Hz)

```
$ rostopic pub /create1/cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}' -r 100
```
