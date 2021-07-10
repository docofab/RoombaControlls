# ROSのサービスとパラメータを理解する

このチュートリアルでは、ROSのサービスとパラメータを紹介し、rosserviceとrosparamコマンドライン・ツールの使い方を説明します。

前回のチュートリアルでturtlesim_nodeがまだ動いていると仮定して、turtlesimが提供するサービスを見てみましょう。

## ROSサービス

サービスは、ノードが相互に通信するためのもう一つの方法です。サービスは、ノードがリクエストを送信し、レスポンスを受信することを可能にします。

## rosserviceの使用

rosserviceは、ROSのクライアント/サービスフレームワークに、サービスを使って簡単にアタッチすることができます。rosserviceには、以下に示すように、サービスに対して使用できる多くのコマンドがあります。

使用方法は以下の通りです。

```
rosservice list アクティブなサービスに関する情報を表示する
rosservice call 提供された引数でサービスを呼び出す
rosservice type サービスの種類を表示します。
rosservice find サービスの種類別にサービスを検索します。
rosservice uri サービスのROSRPC uriを印刷します。
```

### rosservice list

```
$ rosservice list
```

listコマンドによると、turtlesimノードが提供するサービスは、reset、clear、spawn、kill、turtle1/set_pen、/turtle1/teleport_absolute、/turtle1/teleport_relative、turtlesim/get_loggers、turtlesim/set_logger_levelの9つであることがわかります。また、別のrosoutノードに関連する2つのサービスがあります。/rosout/get_loggersと/rosout/set_logger_levelです。

```
/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/teleop_turtle/get_loggers
/teleop_turtle/set_logger_level
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```

rosserviceタイプを使って、クリアサービスをより詳しく見てみましょう。

### rosserviceタイプ

使用方法は以下の通りです。

```
rosservice type [service]
```

クリアサービスのタイプを調べてみましょう。

```
$ rosservice type /clear
std_srvs/Empty
```

このサービスは空です。これは、サービスコールが行われたときに引数を取らないことを意味します（つまり、リクエストを行うときにデータを送信せず、レスポンスを受け取るときにデータを受信しません）。rosservice callを使ってこのサービスを呼び出してみましょう。

### rosserviceコール

使用方法は以下の通りです。

```
rosservice call [service] [args]
```

ここでは、サービスのタイプがemptyであるため、引数を指定せずに呼び出します。

```
$ rosservice call /clear
```

これは期待通り、turtlesim_nodeの背景をクリアします。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams?action=AttachFile&do=get&target=turtlesim.png">

サービスが引数を持つ場合について、サービススポーンの情報を見てみましょう。

```
$ rosservice type /spawn | rossrv show
float32 x
float32 y
float32 theta
string name
---
string name
```

このサービスでは、指定された場所と方向に新しいカメを生成します。nameフィールドは任意なので、新しいカメには名前をつけず、turtlesimに作成してもらいましょう。

```
$ rosservice call /spawn 2 2 0.2 ""
```

このサービスコールは、新しく作られたカメの名前を返します。

```
name: turtle2
```

これで、Turtlesimは次のようになります。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams?action=AttachFile&do=get&target=turtle%28service%29.png">

## rosparamの使用

rosparamを使用すると、ROS Parameter Serverにデータを保存したり操作したりすることができます。パラメータ・サーバーには、整数、浮動小数点、ブーリアン、辞書、およびリストを格納することができます。1は整数、1.0は浮動小数点、oneは文字列、trueは真偽値、[1, 2, 3]は整数のリスト、{a: b, c: d}は辞書です。rosparamには、以下のように、パラメータに対して使用できる多くのコマンドがあります。

使用方法は以下の通りです。

```
rosparam set パラメータの設定
rosparam get パラメータの取得
rosparam load ファイルからパラメータをロードする
rosparam dump パラメータをファイルにダンプする
rosparam delete パラメータの削除
rosparam list パラメータ名のリスト
```

現在、paramサーバにどのようなパラメータがあるかを見てみましょう。

```
rosparam list
```

ここでは、turtlesimノードの背景色に関する3つのパラメータがparamサーバに登録されていることがわかります。

```
/rosdistro
/roslaunch/uris/host_nxt__43407
/rosversion
/run_id
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```

rosparam setを使用して、パラメータ値の1つを変更してみましょう。

### rosparam setとrosparam get

使用方法は以下の通りです。

```
rosparam set [param_name]
rosparam get [param_name]
```

ここでは、背景色の赤チャンネルを変更します。

```
$ rosparam set /turtlesim/background_r 150
```

これでパラメータの値が変更されました。パラメータの変更を有効にするには、clearサービスを呼び出す必要があります。

```
$ rosservice call /clear
```

これでturtlesimは次のようになります。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams?action=AttachFile&do=get&target=turtle%28param%29.png">

次に、paramサーバーの他のパラメータの値を見てみましょう。緑の背景チャンネルの値を取得してみましょう。

```
$ rosparam get /turtlesim/background_g
86
```

また、rosparam get / を使って、Parameter Server全体の内容を表示することもできます。

```
$ rosparam get /
rosdistro: 'noetic

  '
roslaunch:
  uris:
    host_nxt__43407: http://nxt:43407/
rosversion: '1.15.5

  '
run_id: 7ef687d8-9ab7-11ea-b692-fcaa1494dbf9
turtlesim:
  background_b: 255
  background_g: 86
  background_r: 69
```

これをファイルに保存して、別の機会に再読み込みできるようにするとよいでしょう。これはrosparamを使えば簡単です。

### rosparam dumpおよびrosparam load

使用方法は以下の通りです。

```
rosparam dump [file_name] [namespace]
rosparam load [file_name] [namespace]
```

ここでは、すべてのパラメータをparams.yamlというファイルに書き込みます。

```
$ rosparam dump params.yaml
```

これらの yaml ファイルを copy_turtle などの新しい名前空間にロードすることもできます。

```
$ rosparam load params.yaml copy_turtle
$ rosparam get /copy_turtle/turtlesim/background_b
255
```

ROSのサービスとパラメータの仕組みを理解したところで、rqt_consoleとroslaunchを使ってみましょう。


Except where otherwise noted, the ROS wiki is licensed under the [Creative Commons Attribution 3.0](http://creativecommons.org/licenses/by/3.0/)


Wiki: [ROS/Tutorials/UnderstandingServicesParams](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams) (最終更新日時 2020-07-18 21:28:48 更新者 lukelu)

