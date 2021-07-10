# ROSのトピック

このチュートリアルでは、ROSのトピックを紹介するとともに、rostopicやrqt_plotコマンドラインツールの使い方を説明します。

## セットアップ

### roscore

まず、新しいターミナルで、roscoreが起動していることを確認しましょう。

```
$ roscore
```

前回のチュートリアルでroscoreを起動したままにしていると、次のようなエラーメッセージが表示されるかもしれません。

```
roscore cannot run as another roscore/master is already running. 
Please kill other roscore/master processes before relaunching
```

これは問題ありません。roscoreは1つだけ起動すればいいのですから。

### turtlesim

このチュートリアルでは、turtlesimも使用します。新しいターミナルで実行してください。

```
$ rosrun turtlesim turtlesim_node
```

### 亀のキーボード遠隔操作

カメを走らせるためのものも必要です。新しいターミナルで実行してください。

```
$ rosrun turtlesim turtle_teleop_key
Reading from keyboard
---------------------------
Use arrow keys to move the turtle. 'q' to quit.
```

キーボードの矢印キーを使って、亀を走らせることができます。カメを走らせることができない場合は、turtle_teleop_keyのターミナルウィンドウを選択して、入力したキーが記録されていることを確認してください。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle_key.png">

カメを走らせることができたので、今度は舞台裏を見てみましょう。

## ROSトピックス

turtlesim_nodeとturtle_teleop_keyノードは、ROSトピックを介して相互に通信しています。turtle_teleop_keyはキーストロークをトピックに公開し、turtlesimは同じトピックを購読してキーストロークを受け取ります。現在稼働しているノードとトピックを表示するrqt_graphを使ってみましょう。

注：Electric以前のバージョンを使用している場合、rqtは使用できません。代わりにrxgraphを使います。

### rqt_graphの使い方

rqt_graphは、システムで起こっていることを示すダイナミックなグラフを作成します。すでにインストールしていなければ、次のように実行します。

```
$ sudo apt-get install ros-<distro>-rqt
$ sudo apt-get install ros-<distro>-rqt-common-plugins
```

\<distro\>をROSのディストリビューション名に置き換えます（例：indigo、jade、kinetic、lunar ...）。

新しいターミナルで

```
$ rosrun rqt_graph rqt_graph
```

以下のようなものが表示されます。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key.png">


/turtle1/command_velocityの上にマウスを置くと、ROSのノード（ここでは青と緑）とトピック（ここでは赤）がハイライトされます。ご覧のとおり、turtlesim_nodeとturtle_teleop_keyノードは、/turtle1/command_velocityというトピック上で通信しています。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key2.png">

### rostopicの使い方

rostopicツールを使うと、ROSのトピックに関する情報を得ることができます。

helpオプションを使用すると、rostopicで利用可能なサブコマンドを得ることができます。

```
$ rostopic -h
```

```
    rostopic bw トピックが使用する帯域幅を表示します。
    rostopic echo メッセージを画面に表示
    rostopic hz トピックの発行レートを表示します。
    rostopic list アクティブなトピックの情報を表示
    rostopic pub トピックへのデータ発行
    rostopic type トピックの種類を表示します。
```

または、rostopicの後にtabキーを押すと、可能なサブコマンドが表示されます。

```
$ rostopic
bw echo find hz info list pub type
```

これらのトピックサブコマンドのいくつかを使って turtlesim を調べてみましょう。

### rostopic echoの使用

rostopic echoは、あるトピックで公開されているデータを表示します。

使用方法は以下の通りです。

```
rostopic echo [topic]
```

turtle_teleop_keyノードで公開されているコマンドベロシティデータを見てみましょう。

ROS Hydro以降では、このデータは/turtle1/cmd_velトピックで公開されています。新しいターミナルで、次のように実行します。

```
$ rostopic echo /turtle1/cmd_vel
```

ROS Groovy以前のバージョンでは、このデータは/turtle1/command_velocityトピックで公開されています。新しいターミナルで、実行します。

```
$ rostopic echo /turtle1/command_velocity
```

おそらく何も起こらないでしょう。なぜなら、このトピックではデータが公開されていないからです。turtle_teleop_keyが矢印キーを押してデータを発行するようにしてみましょう。もし亀が動かなかったら、もう一度turtle_teleop_keyターミナルを選択する必要があることを覚えておいてください。

ROS Hydro以降では、アップキーを押すと以下のように表示されるようになっています。

```
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
```

ROS Groovy以前のバージョンでは、アップキーを押したときに以下のように表示されるようになりました。

```
---
linear: 2.0
angular: 0.0
---
linear: 2.0
angular: 0.0
---
linear: 2.0
angular: 0.0
---
linear: 2.0
angular: 0.0
---
linear: 2.0
angular: 0.0
```

では、再びrqt_graphを見てみましょう。左上の更新ボタンを押すと、新しいノードが表示されます。ここでは赤で示されているrostopic echoがturtle1/command_velocityのトピックにもサブスクライブされているのがわかります。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_echo.png">

### rostopic listの使用

rostopic listは、現在購読しているすべてのトピックと公開されているトピックのリストを返します。

listサブコマンドが必要とする引数を把握しましょう。新しいターミナルで次のように実行します。

```
$ rostopic list -h
    Usage: rostopic list [/topic]

    Options:
      -h, --help            show this help message and exit
      -b BAGFILE, --bag=BAGFILE
                            list topics in .bag file
      -v, --verbose         list full details about each topic
      -p                    list only publishers
      -s                    list only subscribers
```

rostopic listでは、verboseオプションを使用します。

```
$ rostopic list -v
```

これにより、パブリッシュおよびサブスクライブするトピックとそのタイプの詳細なリストが表示されます。

ROS Hydro以降では、以下のようになります。
```
Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher
 * /rosout [rosgraph_msgs/Log] 2 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 ```

ROS Groovy以前のバージョンの場合。

```
Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/command_velocity [turtlesim/Velocity] 1 publisher
 * /rosout [roslib/Log] 2 publishers
 * /rosout_agg [roslib/Log] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /turtle1/command_velocity [turtlesim/Velocity] 1 subscriber
 * /rosout [roslib/Log] 1 subscriber
 ```
 
## ROSメッセージ

トピックの通信は、ノード間でROSメッセージを送信することで行われます。パブリッシャー(t turtle_teleop_key)とサブスクライバー(turtlesim_node)が通信するためには、パブリッシャーとサブスクライバーが同じタイプのメッセージを送受信する必要があります。つまり、トピックタイプは、その上でパブリッシュされるメッセージタイプによって定義されます。トピックで送信されるメッセージのタイプは、rostopic typeを使用して決定することができます。

### rostopic typeの使い方

rostopic typeは、公開されている任意のトピックのメッセージ・タイプを返します。

使用方法は以下のとおりです。

```
rostopic type [topic]
```

ROS Hydro以降の場合。

    試してみてください。

```
$ rostopic type /turtle1/cmd_vel
```

次の結果が得られるはずです。

```
geometry_msgs/Twist
```

rosmsgを使って、メッセージの詳細を見ることができます。

```
$ rosmsg show geometry_msgs/Twist
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

ROS Groovy以前のバージョンの場合。

試してみてください。

```
$ rostopic type /turtle1/command_velocity
```

次の結果が得られるはずです。

```
turtlesim/velocity
```

rosmsg を使って、メッセージの詳細を見ることができます。

```
$ rosmsg show turtlesim/Belocity
float32 linear
float32 angular
```

turtlesimが期待するメッセージの種類がわかったので、カメにコマンドを発行できるようになりました。

## ROSトピックの続き

ROSのメッセージについて学んだので、メッセージを使ってrostopicを使ってみましょう。

### rostopic pubの使用

rostopic pubは、現在アドバタイズされているトピックにデータをパブリッシュします。

使い方を説明します。

```
rostopic pub [topic] [msg_type] [args].
```

ROS Hydro以降では以下のようになります。

```
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```

ROS Groovy以前のバージョンでは、以下のようになります。

```
$ rostopic pub -1 /turtle1/command_velocity turtlesim/Velocity -- 2.0 1.8
```

前述のコマンドは、直線速度2.0、角速度1.8で動くようにturtlesimに1つのメッセージを送ります。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle%28rostopicpub%29.png">

これはかなり複雑な例なので、各引数を詳しく見てみましょう。

ROS Hydro以降で使用しています。

* このコマンドは、指定されたトピックにメッセージを公開します。

```
rostopic pub
```

* このオプション(dash-one)を指定すると、rostopicはメッセージを1つだけ発行して終了します。

```
-1 
```

* これは公開するトピックの名前です。

```
/turtle1/cmd_vel
```

* これは、トピックに公開するときに使用するメッセージタイプです。

```
geometry_msgs/Twist
```

* このオプション（ダブルダッシュ）は、オプションパーサーに対して、以下の引数のどれもオプションではないことを伝えます。これは、負の数のように、引数の先頭にダッシュ-が付いている場合に必要です。

```
--
```

* 前述したように、geometry_msgs/Twist の msg は、それぞれ 3 つの浮動小数点要素からなる 2 つのベクトル（linear と angular）を持っています。この場合、'[2.0, 0.0, 0.0]'はx=2.0, y=0.0, z=0.0の線形値となり、'[0.0, 0.0, 1.8]'はx=0.0, y=0.0, z=1.8の角型値となります。これらの引数は実際にはYAMLの構文で、YAMLのコマンドライン・ドキュメントで詳しく説明されています。

```
'[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
```

ROS Groovyおよびそれ以前のバージョン用。

* このコマンドは、与えられたトピックにメッセージを公開します。

```
rostopic pub
```

* このオプション(dash-one)を指定すると、rostopicはメッセージを1つだけ発行して終了します。

```
 -1 
```

* これは公開するトピックの名前です。

```
/turtle1/command_velocity
```

* これは、トピックに公開するときに使うメッセージタイプです。

```
turtlesim/Velocity
```

* このオプション(ダブルダッシュ)は、オプションパーサーに、以下の引数のどれもオプションではないことを伝えます。これは、負の数のように、引数の先頭にダッシュ-がついている場合に必要です。

```
--
```

前述のように、turtlesim/Belocityのmsgは、linearとangularの2つの浮動小数点要素を持っています。この場合、2.0がリニア値、1.8がアンギュラー値となります。これらの引数は実際にはYAML構文で、YAMLコマンドライン・ドキュメントで詳しく説明されています。

```
2.0 1.8 
```

これは、カメが動き続けるために、1Hzの定常的なコマンドの流れを必要としているからです。rostopic pub -rコマンドを使って、安定したコマンドの流れを発行することができます。

ROS Hydro以降の場合

```
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

ROS Groovy以前のバージョンの場合。

```
$ rostopic pub /turtle1/command_velocity turtlesim/elocity -r 1 -- 2.0 -1.8
```

これは、velocityトピックに1Hzの速度でvelocityコマンドをパブリッシュします。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle%28rostopicpub%292.png">

また、rqt_graphで起こっていることを見ることもできます。左上の更新ボタンを押してください。rostopic pubノード（ここでは赤）がrostopic echoノード（ここでは緑）と通信しています。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_pub.png">

ご覧のとおり、カメは連続した円を描いて走っています。新しいターミナルでは、rostopic echoを使って、turtlesimが公開したデータを見ることができます。

```
rostopic echo /turtle1/pose
```

### rostopic hzの使用

rostopic hzは、データが公開される割合を報告します。

使い方は以下の通りです。

```
rostopic hz [topic]
```

turtlesim_nodeが/turtle1/poseをどのくらいの速度で公開しているか見てみましょう。

```
$ rostopic hz /turtle1/pose
```

次の結果が得られるはずです。

```
subscribed to [/turtle1/pose]
average rate: 59.354
        min: 0.005s max: 0.027s std dev: 0.00284s window: 58
average rate: 59.459
        min: 0.005s max: 0.027s std dev: 0.00271s window: 118
average rate: 59.539
        min: 0.004s max: 0.030s std dev: 0.00339s window: 177
average rate: 59.492
        min: 0.004s max: 0.030s std dev: 0.00380s window: 237
average rate: 59.463
        min: 0.004s max: 0.030s std dev: 0.00380s window: 290
```

これで、turtlesimが60Hzのレートでカメのデータを公開していることがわかりました。rosmsg showと一緒にrostopic typeを使うことで、トピックに関する詳細な情報を得ることもできます。

ROS Hydro以降の場合

```
$ rostopic type /turtle1/cmd_vel | rosmsg show
```

ROS Groovyおよびそれ以前のバージョンの場合。

```
$ rostopic type /turtle1/command_velocity | rosmsg show
```

rostopicを使ってトピックを調べたので、別のツールを使ってturtlesimで公開されたデータを見てみましょう。

### rqt_plotを使う

注意：Electric以前のバージョンをお使いの場合、rqtは使用できません。代わりにrxplotを使用してください。

rqt_plotは、トピックで公開されているデータをスクロールしてタイムプロットを表示します。ここでは、rqt_plotを使って、「/turtle1/pose」トピックで公開されているデータをプロットしてみます。まず、次のように入力してrqt_plotを起動します。

```
$ rosrun rqt_plot rqt_plot
```

と入力してrqt_plotを起動します。ポップアップした新しいウィンドウでは、左上にテキストボックスがあり、任意のトピックをプロットに追加することができます。turtle1/pose/x」と入力すると、以前は無効だったプラスボタンが強調表示されます。これを押して、同じ手順で「turtle1/pose/y」というトピックを追加します。これで、カメのx-y位置がグラフにプロットされます。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_plot.png">

マイナスボタンを押すと、指定したトピックをプロットから隠すことができるメニューが表示されます。先ほど追加したトピックと、/turtle1/pose/thetaを追加したトピックの両方を非表示にすると、次の図のようなプロットになります。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_plot2.png">

このセクションは以上です。Ctrl-Cを使用して、rostopicターミナルを終了しますが、turtlesimは実行したままです。

ROSトピックの仕組みを理解したところで、サービスとパラメータの仕組みを見てみましょう。

## ビデオチュートリアル

次のビデオは、ROSノードとROSトピック上でturtlesimを使った小さなチュートリアルです。

https://www.youtube.com/embed/Yx_vGAt74sk


Except where otherwise noted, the ROS wiki is licensed under the [Creative Commons Attribution 3.0](https://creativecommons.org/licenses/by/3.0/)

Wiki: [ROS/Tutorials/UnderstandingTopics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics) (最終更新日時 2019-07-18 19:55:02 更新者 AnisKoubaa)
