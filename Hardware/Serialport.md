# シリアルポートの仕様

* 通信速度：115200または19200bps
* データビット：8
* パリティ：なし
* ストップビット：1
* フロー制御：なし

標準では115200bps。115200bpsが使えない場合は、以下の２通りの方法で19200bpsに変更できます。

1. ルンバの電源をオンにするときに、「クリーン/電源」ボタンを押し続けると、10秒後に音階が下がるような曲がながれ、19200bpsに設定されます。
1. ボーレート変更ピン（Mini-DINコネクタのピン5）を使用して、ルンバの通信速度を変更します。ルンバの電源をオンにした後、2秒間待ってから、ボーレート変更ピンを3回ローにパルスします。各パルスは50〜500msの長さが必要です。

19200bpsから115200bpsに戻すには、電源を切るか、OIで通信速度を変更する必要があります。

# Arduinoとの接続

以下のように接続します。

| Arduino | Roomba |
| --- | --- |
| | 1  Vpwr  Roomba battery + (unregulated) |
| | 2  Vpwr  Roomba battery + (unregulated) |
| 11(TX) | 3  RXD  0 – 5V Serial input to Roomba |
| 10(RX) | 4  TXD  0 – 5V Serial output from Roomba |
| | 5 BRC Baud Rate Change |
| GND  | 6 GND  Roomba battery ground | 
| | 7 GND Roomba battery ground |

# Arduinoサンプル

```
#include <SoftwareSerial.h>;
SoftwareSerial serial(10, 11); // RX, TX
void setup() {
  // put your setup code here, to run once:
  serial.begin(115200);
  byte buffer[] = {
    byte(128), // Start Opcode: 128 Data Bytes: 0
    byte(129), // Baud  Opcode: 129 Data Bytes: 1
    byte(7)    //       7: 19200bps
  };
  serial.write(buffer, sizeof(buffer));
}

void loop() {
  // put your main code here, to run repeatedly:

}
```
