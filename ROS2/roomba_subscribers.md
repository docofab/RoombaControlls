# RoombaのSubscribersの確認

## debris_led

DIRT DETECT LEDの点灯(true)、消灯(false)を設定します。

```
$ ros2 topic pub --once /debris_led std_msgs/Bool "data: true"
```

## spot_led

SPOT LEDの点灯(true)、消灯(false)を設定します。

```
$ ros2 topic pub --once /spot_led std_msgs/Bool "data: true"
```

## dock_led

DOCK LEDの点灯(true)、消灯(false)を設定します。

```
$ ros2 topic pub --once /dock_led std_msgs/Bool "data: true"
```

## check_led

CHECK LEDの点灯(true)、消灯(false)を設定します。

```
$ ros2 topic pub --once /check_led std_msgs/Bool "data: true"
```

## power_led

POWER LEDの色と明るさを設定します。データは1または2バイトで、1バイト目は色指定で緑（0）〜赤（255）を指定できます。2バイト目（オプション）は明るさを指定できます。デフォルトは255で最も明るい設定となっています。

```
$ ros2 topic pub --once /power_led std_msgs/UInt8MultiArray "data: [128,255]"
```

## set_ascii

4桁のLED表示を設定します。データは1バイトから4バイトで、ASCIIコードで指定することで左から右に表示されます。小文字のコードは大文字で表示されます。

```
$ ros2 topic pub --once /set_ascii std_msgs/UInt8MultiArray "data: [49,50,51,52]"
```

## define_song

曲の登録を行います。4曲まで登録できます。

song: (0 - 3) 登録する曲の番号
length: (1 – 16) 曲の中の音符の数に応じた、曲の長さ。
notes: (31 – 127) MIDIの音番号
durations: 1音の長さを秒単位で指定します。この値を64倍してルンバのOIに渡しています。

```
$ ros2 topic pub --once /define_song create_msgs/DefineSong "{song: 0,length: 9,notes: [55,55,55,51,58,55,51,58,55],durations: [0.5,0.5,0.5,0.375,0.125,0.5,0.375,0.125,1.0]}"
```

## play_song

define_songで登録した曲(0 - 3) を再生します。

```
$ ros2 topic pub --once /play_song create_msgs/PlaySong "song: 0"
```
