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

## Roomba実機を動かす場合

```
$ roslaunch ca_driver create_2.launch
```
