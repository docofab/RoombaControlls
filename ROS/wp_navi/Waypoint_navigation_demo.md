# ウェイポイントナビゲーションのデモ手順

1. Raspberry Piにログイン
    ```
    ssh ubuntu@192.168.100.36
    ```

1. Raspberry PiでルンバのROSドライバを実行
    ```
    roslaunch ca_bringup minimal2.launch imu:=false
    ```

1. PCでhector_slamを実行
    ```
    roslaunch ca_move_base navigate2.launch localization:=hector_mapping
    ```

1. PCでRvizを起動
    ```
    roslaunch ca_tools rviz.launch
    ```

1. Rvizの2D Nav Goalを使いロボットを移動させ、おまかなmapを作る。

1. mapができたところでウェイポイントのリストを作成するノードを起動する。
    ```
    $ roscd wp_navi/src
    $ rosrun wp_navi wp_goal_sub.py 
    ```

1. ウェイポイントをRvizで指定しながらロボットを動かし、そのゴールデータをlistに落とす
    ```
    （例）
    [(1.67217516899,-0.108617782593,0.0),(0.0,0.0,-0.706421072541,0.707791825518)],
    [(0.671614408493,-0.68189907074,0.0),(0.0,0.0,0.743972727,0.668209983075)],
    [(0.528070449829,0.025830745697,0.0),(0.0,0.0,0.488727769111,0.872436340199)],
    ```

1. 作成したリストをwp_navi2.pyに組み込む
    ```
    $ roscd wp_navi/src
    $ vi wp_navi2.py
    ```

1. ウェイポイントナビゲーションを実行する。
    ```
    $ roscd wp_navi/src
    $ rosrun wp_navi wp_navi2.py
    ```

1. リスト通りにウェイポイントナビゲーションが行われる。

