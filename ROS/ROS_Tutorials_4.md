# ROSのノードについて

このチュートリアルでは、ROSのグラフの概念を紹介し、roscore、rosnode、rosrunコマンドラインツールの使用について説明します。

## 前提条件

このチュートリアルでは lighweight シミュレータを使用し、以下のコマンドを実行してインストールします。

```
$ sudo apt-get install ros-<distro>-ros-tutorials
```
\<distro\>はROSのディストリビューションの名前に置き換えてください（例：indigo、jade、kinetic）。

## グラフ概念の簡単な概要

* ノード: ノードとは、ROSを使って他のノードと通信する実行ファイルのことです。
* メッセージ: トピックへのサブスクライブやパブリッシュの際に使用されるROSのデータタイプ。
* トピック: ノードは、メッセージをトピックに公開したり、メッセージを受信するためにトピックをサブスクライブしたりすることができます。
* マスター: ROSのネームサービス（ノードがお互いに見つけられるようにする）。
* rosout: ROSのstdout/stderrに相当します。
* roscore: Master + rosout + parameter server (parameter serverは後で紹介します) 

## ノード

ノードは、ROSパッケージ内の実行ファイルに過ぎません。ROSノードはROSクライアントライブラリを使って他のノードと通信します。ノードはTopicを公開したり購読したりすることができます。また、ノードはサービスを提供したり利用したりすることができます。

## クライアントライブラリ

ROSクライアントライブラリは、異なるプログラミング言語で書かれたノードの通信を可能にします。

* rospy = pythonクライアントライブラリ
* roscpp = c++クライアントライブラリ 

## roscore

roscoreは、ROSを使用する際に最初に実行すべきものです。

実行してください。

```
$ roscore
```

次の結果が得られるはずです。

```
... logging to ~/.ros/log/9cf88ce4-b14d-11df-8a75-00251148e8cf/roslaunch-machine_name-13039.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://machine_name:33919/
ros_comm version 1.4.7

SUMMARY
======

PARAMETERS
 * /rosversion
 * /rosdistro

NODES

auto-starting new master
process[master]: started with pid [13054]
ROS_MASTER_URI=http://machine_name:11311/

setting /run_id to 9cf88ce4-b14d-11df-8a75-00251148e8cf
process[rosout-1]: started with pid [13067]
started core service [/rosout]
```

roscoreが初期化されない場合は、ネットワークの設定に問題がある可能性があります。ネットワーク設定 - シングルマシン構成を参照してください。

roscoreが初期化されず、パーミッション不足のメッセージが出た場合、\~/.rosフォルダの所有者がrootになっている可能性がありますので、そのフォルダの所有者を再帰的に変更してください。

```
$ sudo chown -R <your_username> ~/.ros
```

## rosnodeの使用

新しいターミナルを開き、rosnodeを使ってroscoreの実行結果を見てみましょう。新しいタブを開くか、単純に最小化するかして、前のターミナルを開いておくことに注意してください。

注意：新しいターミナルを開くと、環境がリセットされ、\~/.bashrcファイルがソースされます。rosnodeのようなコマンドの実行に問題がある場合は、\~/.bashrcに環境設定ファイルを追加するか、手動で再ソースする必要があるかもしれません。

rosnodeは、現在動作しているROSノードに関する情報を表示します。rosnode listコマンドは、これらのアクティブなノードをリストアップします。

```
$ rosnode list
```

次の結果が得られるはずです。

```
/rosout
```

これは、実行中のノードはrosoutの1つだけであることを示しています。これは、ノードのデバッグ出力を収集してログに記録するため、常に実行されています。

rosnode infoコマンドは、特定のノードに関する情報を返します。

```
$ rosnode info /rosout
```

これにより、rosoutが/rosout_aggを発行していることなど、rosoutに関する情報が得られました。

```
------------------------------------------------------------------------
Node [/rosout]
Publications:
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions:
 * /rosout [unknown type]

Services:
 * /rosout/get_loggers
 * /rosout/set_logger_level

contacting node http://machine_name:54614/ ...
Pid: 5092
```

さて、さらにいくつかのノードを見てみましょう。そのために、rosrunを使って別のノードを立ち上げてみます。

## rosrun の使用

rosrunでは、パッケージ名を使ってパッケージ内のノードを直接実行することができます（パッケージのパスを知らなくても大丈夫です）。

使い方は以下の通りです。

```
$ rosrun [package_name] [node_name]
```

これで、turtlesim パッケージ内の turtlesim_node を実行することができます。

次に、新しいターミナルで

```
$ rosrun turtlesim turtlesim_node
```

すると、turtlesimのウィンドウが表示されます。

<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes?action=AttachFile&do=get&target=turtlesim.png">

注： turtlesimのウィンドウでは、カメが違って見えるかもしれません。気にしないでください。亀にはたくさんの種類がありますし、あなたの亀も驚きです。

新しいターミナルで

```
$ rosnode list
```

次の結果が得られるはずです。

```
/rosout
/turtlesim
```

ROSの強力な機能の一つに、コマンドラインから名前を再割り当てできることがあります。

turtlesimウィンドウを閉じてノードを停止します（またはrosrun turtlesimターミナルに戻ってctrl-Cを使用してください）。それでは再実行してみましょう。今度はRemaping Argumentを使ってノードの名前を変更します。

```
$ rosrun turtlesim turtlesim_node __name:=my_turtle
```

さて、戻ってrosnode listを使ってみましょう。

```
$ rosnode list
```

次の結果が得られるはずです。

```
/my_turtle
/rosout
```

注: リストにまだ/turtlesimが表示されている場合、ターミナルでウィンドウを閉じる代わりにctrl-Cを使ってノードを停止したか、「Network Setup - Single Machine Configuration」で説明されているように$ROS_HOSTNAME環境変数が定義されていないことを意味しているかもしれません。以下の方法でrosnodeリストをクリーンアップしてみてください。$ rosnode cleanup

新しい/my_turtleノードが表示されます。別のrosnodeコマンドであるpingを使用して、それが起動していることをテストしてみましょう。

```
$ rosnode ping my_turtle
rosnode: node is [/my_turtle]
pinging /my_turtle with a timeout of 3.0s
xmlrpc reply from http://aqy:42235/     time=1.152992ms
xmlrpc reply from http://aqy:42235/     time=1.120090ms
xmlrpc reply from http://aqy:42235/     time=1.700878ms
xmlrpc reply from http://aqy:42235/     time=1.127958ms
```

## まとめ

カバーした内容

* roscore = ros+core : master (ROSのネームサービスを提供) + rosout (stdout/stderr) + parameter server (parameter serverは後で紹介します)
* rosnode = ros+node : ノードの情報を得るためのROSツールです。
* rosrun = ros+run : 指定されたパッケージからノードを実行します。

ROSのノードの動作を理解したところで、ROSのトピックの動作を見てみましょう。また、turtlesim_nodeを停止するにはCtrl-Cを自由に押してください。

Except where otherwise noted, the ROS wiki is licensed under the Creative Commons Attribution 3.0

Wiki: [ROS/Tutorials/UnderstandingNodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes) (最終更新日時 2019-06-05 23:41:12 更新者 LukeMeier)
