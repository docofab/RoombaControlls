# DUALSHOCK4を使う手順

## ドライバのインストール

1. 以下の手順でドライバをインストールします。
    ```
    pip install ds4drv
    ```

## DUALSHOCK4をペアリングモードにする

1. DUALSHOCK4のPSボタンとSHAREボタンを同時に長押しする。
1. LEDがチカッと短く点灯を繰り返す状態になったらペアリングモードに入った合図です。

## ドライバを動かす

1. ドライバを実行します。
    ```
    sudo ds4drv
    ```
1. 正常に動作すれば以下のようにペアリンクします。
    （動作例）
    ```
    $ sudo ds4drv
    [info][controller 1] Created devices /dev/input/js0 (joystick) /dev/input/event20 (evdev) 
    [info][bluetooth] Scanning for devices
    [info][bluetooth] Found device 4C:B9:9B:19:24:7F
    [info][controller 1] Connected to Bluetooth Controller (4C:B9:9B:19:24:7F)
    [info][bluetooth] Scanning for devices
    [info][controller 1] Battery: Fully charged

1. うまくペアリングしない場合は以下のコマンドを入力してみる。
    ```
    sudo hciconfig hciX up
    ```

## 動作確認を行う

1. 新しいターミナルを起動し、roscoreを動かします。
    ```
    roscore
    ```
1. 新しいターミナルを起動し、joyノードを動かします。
    ```
    rosrun joy joy_node
    ```
    （動作例）
    ```
    $ rosrun joy joy_node
    [ WARN] [1645322416.798315279]: Couldn't set gain on joystick force feedback: Bad file descriptor
    [ INFO] [1645322416.800946236]: Opened joystick: /dev/input/js0. deadzone_: 0.050000.
    ```
1. 新しいターミナルを起動し、トピックを確認します。
    ```
    rostopic echo /joy
    ```
    （動作例）
    ```
    $ rostopic echo /joy
    header: 
      seq: 1364
      stamp: 
        secs: 1645322443
        nsecs: 509287502
      frame_id: "/dev/input/js0"
    axes: [0.0, 0.0, -0.0, 0.0, 0.0, 0.0, -0.0, 0.32399895787239075, -0.3271471858024597, 0.0, 0.0, -0.0, -0.0, -0.0]
    buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ---
    header: 
      seq: 1365
      stamp: 
        secs: 1645322443
        nsecs: 531815111
      frame_id: "/dev/input/js0"
        :
    ```

## DUALSHOCK4の電源を切る

1. DUALSHOCK4のPSボタンを10秒間長押しする。