# ルンバ600系の簡易ユーザーマニュアル

## バッテリー関連
- バッテリーアイコン
  - オレンジ色点滅：充電中
  - 緑色点灯：充電完了
- バッテリー残量が少なくなると、CLEANボタンが赤く点滅するので充電を行うこと。
- 節電のため、充電開始から60秒後にすべてのランプが消灯する。

## 言語設定
1. ホームベースから外し、CLEANボタンを長押しして電源を切ります。
2. そのままCLEANボタンを押したままにすると、今設定されている言語をしゃべりますので、CLEANボタンから手を離します。
3. さらにCLEANボタンを1回押すと設定言語が切り替わり、その言語でしゃべります。
4. 希望の言語になったら、CLEANボタンを長押しして電源を切ります。

## エラーの時
- エラーアイコンが点滅したときは、http://www.irobot-jp.com/support/ を参照のこと。

## ファームウェアのバージョン確認

### 電源ボタンによる確認方法

1. ルンバのシリアルポートにUSBシリアルを接続します。
1. ターミナルソフトを起動し、通信速度を115200bps, Data: 8bit, Parity:none, Stop: 1bit, Flow: Noneにします。
1. 電源を投入すると以下のようなメッセージがターミナルに表示されます。

    ```
    key-wakeup
    slept for 0 minutes 14 seconds
    
    2015-08-24-1648-L
    r3-robot/tags/release-3.5.x-tags/release-3.5.4:6058 CLEAN
    
    bootloader id: 470E 6360 83EB FFFF
    assembly: 3.5-lite-batt
    revision: 2
    flash version: 10
    flash info crc passed: 1
    
    battery-current-zero 262
    ```

### TeraTermでRESETコマンド送信による確認方法

1. reset.ttlというファイルを作成し、以下の内容で保存します。
    ```
    send $07
    ```
1. ルンバのシリアルポートにUSBシリアルを接続します。
1. TeraTeamを起動し、通信速度を115200bps, Data: 8bit, Parity:none, Stop: 1bit, Flow: Noneにします。
1. TeraTeamのコントロール->マクロでreset.ttlを指定します。
1. 以下のようなメッセージがターミナルに表示されます。

    ```
    bl-start
    STR730
    bootloader id: #x470E6360 83EBFFFF
    bootloader info rev: #xF000
    bootloader rev: #x0001
    2007-05-14-1715-L
    Roomba by iRobot!
    str730
    2015-08-24-1648-L
    battery-current-zero 261
    bbox vars restored!
    languages: japanese (13)
    ```
