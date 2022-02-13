# Roombaのシミュレーション環境でのSLAMとNavigation

## STEP1. シミュレータ環境の準備

Roombaのシミュレーション環境は以下の４つのパターンがあるので利用できるものを準備する。

* パターン1　Raspberry Pi 4にUbuntu+ROSをいれてLinux PC替わりに使う。（ちと重い）
* パターン2  Dockerで動かす。(ただし、M1 MacはGazeboが動かない）
* パターン3  VMWare workstation playerにUbuntu+ROSをいれて動かす。（ただし、M1 MacはGazeboが動かない）
* パターン4  UbuntuをインストールしたPCでROSを動かす。（一番高速） 

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
1. 新しいターミナルで以下を入力
    ```
    roslaunch ca_tools keyboard_teleop.launch
    ```
1. キーボードを動かして地図を作る。

## STEP3. 地図の保存

1. 地図ができたところで、新しいターミナルを起動して以下を入力する
    ```
    rosrun map_server map_saver -f mymap
    ```
1. カレントディレクトリに mymap.pgm と mymap.yaml というファイル名で地図情報が保存される。

## STEP4. 作成したWorldと地図を配置する。

1. 作成したmymap.worldファイルを所定のディレクトリに配置する。

    ```
    cp ~/mymap.world ~/catkin_ws/src/create_autonomy/ca_gazebo/worlds/.
    ```

1. 作成した地図ファイル(mymap.pgm, mymap.yaml)を所定のディレクトリに配置する。
    ```
    cp ~/mymap.pgm ~/mymap.yaml ~/catkin_ws/src/create_autonomy/navigation/ca_move_base/maps
    ```

1. envパラメタにmymapを指定してgazeboを起動する。（mymapと指定すると自動的にmymap.world, mymap.pgm, mymap.yamlが読み込まれる）
    ```
    export LOCALIZATION=amcl
    roslaunch ca_gazebo create_empty_world.launch env:=mymap
    ```




