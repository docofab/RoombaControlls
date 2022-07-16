# よくつかうコマンド一覧

## ノードの一覧を見る
```
$ ros2 node list
```

## トピックの一覧を見る

```
$ ros2 topic list
```

## トピックの一覧をGUIで表示
```
$ rqt
```

## Nodeとtopicを図で表示
```
$ rqt_graph
```

## tfのフレーム表示
```
$ ros2 run tf2_tools view_frames.py
```

## 通信確認
トピックの送信・受信確認
```
ros2 run demo_nodes_cpp talker

ros2 run demo_nodes_cpp listener
```
マルチキャストUDPパケットの通信確認
```
ros2 multicast receive

ros2 multicast send
```

## ROS2 daemon
```
ros2 daemon status
ros2 daemon start
ros2 daemon stop
```
