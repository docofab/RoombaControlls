# Roombaの実機環境でのSLAMとNavigation

* 参考ページ：https://demura.net/robot/hard/20061.html

## STEP1. Roomba実機環境の準備

1. Raspberry Pi 3をRoombaに接続し、キーボードで操作できることを確認します。
    * [setup-real-roomba-rasppi_3.md](setup-real-roomba-rasppi_3.md)

## STEP2. 地図の取得

1. リモートPCで新しいターミナルを開き、以下のコマンドを入力します。
    １つめのターミナル
    ```
    export ID=1
    export IMU=false
    export LOCALIZATION=slam
    export LASER=rplidar
    roslaunch ca_bringup complete.launch
    ```
    ２つめのターミナル
    ```
    export RVIZ_CONFIG=navigation
    roslaunch ca_tools rviz.launch
    ```
    ３つめのターミナル
    ```
    roslaunch ca_tools keyboard_teleop.launch
    ```

1. Rvizを見ながらキーボードをルンバを丁寧に動かして地図を作ります。

## STEP3. 地図の保存

1. 地図ができたところで、新しいターミナルを起動して以下を入力する
    ```
    rosrun map_server map_saver -f mymap
    ```
1. カレントディレクトリに mymap.pgm と mymap.yaml というファイル名で地図情報が保存される。

## STEP4. 作成した地図を配置する。

1. 作成した地図ファイル(mymap.pgm, mymap.yaml)を所定のディレクトリに配置する。
    ```
    cd
    cp mymap.pgm mymap.yaml ~/catkin_ws/src/create_autonomy/navigation/ca_move_base/maps/
    ```
## STEP5. 作成した地図でNavigationをする。

1. envパラメタにmymapを指定してRvizを起動する。mymap.pgm, mymap.yamlが読み込まれ、Rvizにmapが表示される。
    ```
    export LOCALIZATION=amcl
    export RVIZ=true
    export LASER=rplidar
    export RVIZ_CONFIG=navigation
    roslaunch ca_move_base navigate.launch env:=mymap
    ```
1. RvizのPanels -> Tool Propertiesにチェックを入れる。
1. Tool Propertiesパネルが表示されるので、2D Pose EstimateのTopicを /initialpose から /create1/initialpose に変更し、2D Nav GoalのTopicを /move_base_simple/goal から /create1/move_base_simple/goal に変更する。
1. Rvizの2D Pose Estimationをクリックして、実際のルンバの位置をマウスポインタで指定し、緑の矢印をルンバの向きに合うようにドラッグすることで、大まかな位置を合わせる。
1. Rvizを見ながらkeyboard_teleop.launchのターミナルからキーボードを操作してルンバの位置を上下左右や回転などを行い少しずつ動かすと、ルンバはセンサの情報を収集し、地図上の位置が同定される。

1. 地図上の位置が同定されたあとに、Rvizの2D Nav Goalをクリックし、移動先の地点を指定するとルンバがその場所まで移動する。
