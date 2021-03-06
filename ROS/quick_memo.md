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
## turtlesimの亀をリセットする

```
$ rosservice call reset
```

## Gazeboの起動

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

## トピックの一覧を見る

```
$ rostopic list
```

## トピックの一覧をGUIで表示
```
$ rqt
```

## 複数パッケージの起動

```
$ roslaunch <パッケージ名>　<Launchファイル名>
```
例: ca_gazebo パッケージの　create_empty_world.launch ファイル
```
$ roslaunch ca_gazebo create_empty_world.launch
```

## Nodeとtopicを図で表示
```
$ rqt_graph
```

## tfの内容の表示
```
$ rosrun tf tf_echo turtle3 turtle1
```

## tfのフレーム表示
```
$ rosrun tf view_frames
```

## Ubuntu server for Raspberry Pi
* TIMEZONEの設定
```
$ sudo timedatectl set-timezone Asia/Tokyo 
```
* 日本語キーボードの設定
```
sudo dpkg-reconfigure keyboard-configuration
```
* NTPの確認
```
systemctl status systemd-timesyncd.service
systemctl is-enabled systemd-timesyncd.service
```
