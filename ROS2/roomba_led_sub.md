# RoombaのLED制御

## debris_led

DIRT DETECT LEDの点灯(true)、消灯(false)を設定します。

```
$ ros2 topic pub --once /debris_led std_msgs/msg/Bool "data: true"
```

## spot_led

SPOT LEDの点灯(true)、消灯(false)を設定します。

```
$ ros2 topic pub --once /spot_led std_msgs/msg/Bool "data: true"
```

## dock_led

DOCK LEDの点灯(true)、消灯(false)を設定します。

```
$ ros2 topic pub --once /dock_led std_msgs/msg/Bool "data: true"
```

## check_led

CHECK LEDの点灯(true)、消灯(false)を設定します。

```
$ ros2 topic pub --once /check_led std_msgs/msg/Bool "data: true"
```

## power_led

POWER LEDの色と明るさを設定します。データは1または2バイトで、1バイト目は色指定で緑（0）〜赤（255）を指定できます。2バイト目（オプション）は明るさを指定できます。デフォルトは255で最も明るい設定となっています。

```
$ ros2 topic pub --once /power_led std_msgs/msg/UInt8MultiArray "data: {128,255}"
```

## set_ascii

4桁のLED表示を設定します。データは1バイトから4バイトで、ASCIIコードで指定することで左から右に表示されます。

```
$ ros2 topic pub --once  /set_ascii std_msgs/msg/UInt8MultiArray "data: {49,50,51,52}"
```
