# ルンバの動作モードについて

## ルンバOIの動作モード

ルンバOIにはOff, Passive, Safe, Fullの4つの動作モードがあります。

## Offモード

* モード切り替え方法
  * 電池交換後、または初めて電源を入れたとき
  
* OFFモードでの動作
  * 標準ボーレート（115200または19200bps）でOIスタートコマンドを待ちます。
  * スタートコマンドを受信すると、OIにモードコマンドを送信することにより、4つの動作モードのいずれかに入ることができます。
  * 使用したい動作モードのコマンドをOIに送信することで、いつでも動作モードを切り替えることができます。

## Passiveモード

* モード切り替え方法
  * スタート・コマンドまたはクリーニング・モード・コマンドのいずれか（例：Spot、Clean、Seek Dock）を送信する。

* パッシブモードでの動作
  * センサー・データを要求・受信
  * アクチュエータ（モーター、スピーカー、ライト）のパラメタは変更できない。
  * バッテリーの充電は可能。

## Safeモード

* モード切替方法
  * Safeコマンドを送信する
  
* Safeモードの動作
  * 以下の安全に関する条件を除き、ルンバを完全に制御できます。
    - 前進中に崖を発見した場合（またはロボットの半径以下の小さな回転半径で後退した場合）
    - どれかの車輪の落下を検知した場合
    - 充電器が接続され、電源が入っている場合

    Safeモード時に上記の状態になると、ルンバはすべてのモーターを停止し、パッシブモードに戻ります。
  * Safeモード時にOIにコマンドが送られない場合、ルンバはすべてのモーターとLEDをオフにして待機し、ボタンなどのセンサー入力にも反応しません。
  * Safeモードに入ると充電が終了します。

## Fullモード

* モード切り替え方法
  * Fullコマンドを送信する。
  
* Fullモードの動作
  * Safeモードで機能していた安全機能が停止します。
  * Safeモードでの制限が解除され、ルンバのすべてのアクチュエータ、および安全関連のすべての条件を完全に制御することができます。
  * Safeモードに戻すには、Safe コマンドを送信する必要があります。
  * Fullモード時にOIにコマンドを送信しない場合、すべてのモーターとLEDをオフにして待機し、ボタンを押したり他のセンサー入力に反応しません。
  * Fullモードに入ると充電が終了します。

# Arduinoサンプル

```
#include <SoftwareSerial.h>;
SoftwareSerial serial(10, 11);  // RX, TX
void setup() {
  // put your setup code here, to run once:
  serial.begin(115200);
  byte buffer[] = {
    byte(128), // Start Opcode: 128 Data Bytes: 0
    byte(131)  // Safe  Opcode: 131 Data Bytes: 0 
  };
  serial.write(buffer, sizeof(buffer));
}

void loop() {
  // put your main code here, to run repeatedly:

}
```