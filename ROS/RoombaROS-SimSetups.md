Roombaのシミュレーション環境のセットアップ
===============================

参考サイト
``` bash
### ROSのDockerシミュレータ
Docker上でGUIのROS1/ROS2を一瞬でセットアップする方法
https://qiita.com/karaage0703/items/957bdc7b4dabfc6639da
ROS/ROS2のGUIをWebブラウザ経由でお手軽に試せるDockerfileを公開しました
https://memoteki.net/archives/2955
https://github.com/Tiryoh/docker-ros-desktop-vnc
### CreateAutonomy
HARD2021: Gazeboシミュレータでルンバを動かそう！
https://demura.net/robot/hard/20405.html

```


## 1.Docker を入れる

### Windows

1.1.win WSL2 を導入する

1.2.winWindows 10 Home で WSL 2 + Docker を使う
https://qiita.com/KoKeCross/items/a6365af2594a102a817b

#### ※wslのメモリ制限の設定

Powershellで
``` bash
code %USERPROFILE%\.wslconfig
``` 
として、.wslconfigファイルを作成
``` bash
[wsl2]
memory=3GB
processors=2
``` 
を記載する。(記載はプロセッサー数も制限している)

``` bash
wsl --shutdown
``` 


参考：WSL2のメモリ割り当て量を変えたい
https://qiita.com/Ischca/items/121d91eb3b1a0a1cd8a8

### Linux
``` bash

``` 

## 2. 既存イメージをpullする



## 3. Roombaの操作ライブラリをインストールする。

## 4. ビルドを通す

## 5. 完成


