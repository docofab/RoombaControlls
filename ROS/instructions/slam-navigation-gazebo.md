# Roombaのシミュレーション環境でのSLAMとNavigation

* 参考ページ：https://demura.net/robot/hard/20061.html

## STEP1. シミュレータ環境の準備

Roombaのシミュレーション環境は以下の４つのパターンがあるので利用できるものを準備する。

* パターン1 [Raspberry Pi 4にUbuntu+ROSをいれてLinux PC替わりに使う。](../instructions/setup-gazebo-rasppi.md)（ちと重い）
* パターン2 [Dockerで動かす。](../instructions/setup-gazebo-docker.md) (ただし、M1 MacはGazeboが動かない）
* パターン3 [VMWare workstation playerにUbuntu+ROSをいれて動かす。](../instructions/setup-gazebo-vmware.md)（ただし、M1 MacはGazeboが動かない）
* パターン4 [UbuntuをインストールしたPCでROSを動かす。](../instructions/setup-gazebo-ubuntu.md)（一番高速） 

## STEP2. 新しいWorldの作成と地図の取得

1. 空のWORLDでGazeboとRvizを起動
    ```
    cd
    export LOCALIZATION=slam
    export RVIZ_CONFIG=navigation
    roslaunch ca_gazebo create_empty_world.launch
    ```
1. Gazeboで角柱、円柱、球を適当に配置する。
1. 配置が完了した状態で、GazeboのFile -> Save World As で mymap.world という名前で保存する。カレントディレクトリに mymap.world が保存される。
1. 新しいターミナルを起動して以下を入力
    ```
    roslaunch ca_tools keyboard_teleop.launch
    ```
1. Rvizを見ながらキーボードをルンバを丁寧に動かして地図を作る。

## STEP3. 地図の保存

1. 地図ができたところで、新しいターミナルを起動して以下を入力する
    ```
    rosrun map_server map_saver -f mymap
    ```
1. カレントディレクトリに mymap.pgm と mymap.yaml というファイル名で地図情報が保存される。

## STEP4. 作成したWorldと地図を配置する。

1. 作成したmymap.worldファイルを所定のディレクトリに配置する。

    ```
    cp ~/mymap.world ~/catkin_ws/src/create_autonomy/ca_gazebo/worlds
    ```

1. 作成した地図ファイル(mymap.pgm, mymap.yaml)を所定のディレクトリに配置する。
    ```
    cp ~/mymap.pgm ~/mymap.yaml ~/catkin_ws/src/create_autonomy/navigation/ca_move_base/maps
    ```
## STEP5. 作成したWorldと地図でNavigationをする。

1. envパラメタにmymapを指定してGazeboとRvizを起動する。mymap.world, mymap.pgm, mymap.yamlが読み込まれ、Gazeboにworldが、Rvizにmapが表示される。
    ```
    export LOCALIZATION=amcl
    export RVIZ_CONFIG=navigation
    roslaunch ca_gazebo create_empty_world.launch env:=mymap
    ```
1. RvizのPanels -> Tool Propertiesにチェックを入れる。
1. Tool Propertiesパネルが表示されるので、2D Pose EstimateのTopicを /initialpose から /create1/initialpose に変更し、2D Nav GoalのTopicを /move_base_simple/goal から /create1/move_base_simple/goal に変更する。
1. Rvizの2D Pose Estimationをクリックして、実際のルンバの位置をマウスポインタで指定し、緑の矢印をルンバの向きに合うようにドラッグすることで、大まかな位置を合わせる。
1. Rvizを見ながらkeyboard_teleop.launchのターミナルからキーボードを操作してルンバの位置を上下左右や回転などを行い少しずつ動かすと、ルンバはセンサの情報を収集し、地図上の位置が同定される。

1. 地図上の位置が同定されたあとに、Rvizの2D Nav Goalをクリックし、移動先の地点を指定するとルンバがその場所まで移動する。
