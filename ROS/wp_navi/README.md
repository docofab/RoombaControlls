# おおたfab mapでのナビゲーション

## Gazebo

1. Gazeboの起動
    ```
    export LOCALIZATION=amcl
    roslaunch ca_gazebo create_otafab1.launch
    ```
1. ウェイポイントナビゲーションの実行
    ```
    roslaunch wp_navi wp_navi_otafab1.launch
    ```
1. ウェイポイントの表示
    ```
    roscd wp_navi/src
    rosrun wp_navi waypoint.py 
    ```
    RvizでAdd->By topic->/Waypoint Markerを追加

## Real

1. ca_bringupの起動(Raspberry pi)
    ```
    roslaunch ca_bringup minimal2.launch
    ```
1. ナビゲーションの起動
    ```
    roslaunch ca_move_base navigate2.launch env:=map localization:=amcl 
    ```
    使用するmapはenvパラメタで指定。otafabはmap。
1. 現在座標の設定
    - Rvizの2D Pose Estimateで現在位置と方向を指定
1. 目標座標の設定（テスト）
    - 2D Nav Goalで目標位置と方向を指定。
1. ウェイポイントナビゲーションの実行
    ```
    roslaunch wp_navi wp_navi_otafab1.launch
    ```
1. ウェイポイントの表示
    ```
    roscd wp_navi/src
    rosrun wp_navi waypoint.py 
    ```
