# ROSの動作確認

## Raspberry Piでとりあえずわかったことまとめ
* ルンバを動かすためのcreate_autonomyはROS Noeticには対応していないので、ROS Melodic が必要。
* ROS MelodicはUbuntu 18.04しか動かない。http://wiki.ros.org/melodic/Installation/Ubuntu
* Raspberry Pi 3ではメモリ不足で動作が厳しい。
* Raspberry Pi 3用のubuntu mate 18.04はRaspberry Pi 4では起動しない。
* Raspberry Pi 4でUbuntu 18.04を動かすには Ubuntu 18.04 Server (ARM版)を使い、デスクトップ環境はあとから追加する。https://demura.net/education/17957.html
* ROS Melodicのインストールまではできる。https://demura.net/robot/16518.html
* Gazeboシミュレータは動いた。https://demura.net/robot/hard/20405.html

## 動作確認結果(2021/6/19現在)

### ROS Melodic

| HOST | GuestOS | ROS Melodic | Gazebo | Notes |
|------|------------|--------|-----|----|
| Raspberry Pi4 | Ubuntu 18.04 LTS Server + desktop| OK | OK |  |
| Windows10 | Ubuntu 18.04 LTS(VMware Player 16) | OK | OK |export SVGA_VGPU10=0の設定が必要|
| Widnows10 | Docker Desktop | OK | OK |  |
| Mac M1 | Docker Desktop | NG(turtleでsegv) | NG(segv) |  |

※Mac M1のParallels Desktop 16ではUbuntu18.04はサポートしていない。

### ROS2 Foxy

| HOST | GuestOS | ROS2 Foxy | Webots | Notes |
|------|------------|--------|-----|----|
| Raspberry Pi4 | Ubuntu Mate 20.03 |未確認 | 未確認 |  |
| Windows10 | Ubuntu 20.03LTS(VMware Player 16) | 未確認 | 未確認 |  |
| Widnows10 | Docker Desktop | OK | OK |  |
| Mac M1 | Ubuntu 20.03(Parallels Desktop 16) | NG | NG |  |
| Mac M1 | Docker Desktop | NG | NG |  |
