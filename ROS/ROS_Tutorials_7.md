# rqt_consoleとroslaunchの使用について

このチュートリアルでは、デバッグのためのrqt_consoleとrqt_logger_level、および多数のノードを一度に起動するためのroslaunchを使ってROSを紹介します。

## rqtおよびturtlesimパッケージのインストール

このチュートリアルでは、rqtとturtlesimの両方のパッケージを使用します。このチュートリアルを行うために、まだ両パッケージをインストールしていない場合は、インストールしてください。

```
$ sudo apt-get install ros-<distro>-rqt ros-<distro>-rqt-common-plugins ros-<distro>-turtlesim
```

\<distro\>はお使いのROSディストリビューションの名前に置き換えてください（例：melodic）。

注意：以前のチュートリアルでrqtとturtlesimをすでに構築しているかもしれません。確信が持てない場合は、再度インストールしても何の問題もありません。

## rqt_consoleとrqt_logger_levelを使う

rqt_consoleはROSのロギングフレームワークに取り付けて、ノードからの出力を表示します。rqt_logger_levelでは、ノードの冗長度（DEBUG、WARN、INFO、ERROR）を実行中に変更することができます。

では、rqt_consoleでturtlesimの出力を見て、rqt_logger_levelでロガーレベルを切り替えながらturtlesimを使ってみましょう。turtlesimを起動する前に、2つの新しいターミナルでrqt_consoleとrqt_logger_levelを起動します。

1つめのターミナル

```
$ rosrun rqt_console rqt_console
```

2つめのターミナル

```
$ rosrun rqt_logger_level rqt_logger_level
```

2つのウィンドウがポップアップするのがわかります。

<img src="http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch?action=AttachFile&do=get&target=rqt_console%28start%29.png">

<img src="http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch?action=AttachFile&do=get&target=rqt_logger_level.png">

それでは、新しいターミナルでturtlesimを起動してみましょう。

```
$ rosrun turtlesim turtlesim_node
```

デフォルトのロガーレベルはINFOなので、turtlesimの起動時に公開される情報がすべて表示されます。

<img src="http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch?action=AttachFile&do=get&target=rqt_console%28turtlesimstart%29.png">

では、以下のようにrqt_logger_levelウィンドウのノードを更新して「Warn」を選択し、ロガーレベルを「Warn」に変更しましょう。

<img src="http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch?action=AttachFile&do=get&target=rqt_logger_level%28error%29.png">

では、カメを壁に走らせて、rqt_consoleに何が表示されるか見てみましょう。

```
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

<img src="http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch?action=AttachFile&do=get&target=rqt_console%28turtlesimerror%29.png">

### ロガーレベルに関するクイックノート

ロガーレベルは以下の順番で優先されます。

```
Fatal
Error
Warn
Info
Debug
```

Fatal が最も優先度が高く、Debug が最も優先度が低いです。ロガーレベルを設定することで、その優先度以上のメッセージをすべて取得することができます。例えば、レベルをWarnに設定すると、Warn、Error、Fatalのすべてのロギングメッセージが表示されます。

Ctrl-C で turtlesim を起動し、roslaunch で複数の turtlesim ノードと、ある turtlesim を別の turtlesim に模倣させる mimicking ノードを起動してみましょう。

### roslaunchの使い方

roslaunch は launch ファイルで定義されたノードを起動します。

使い方は以下の通りです。

```
$ roslaunch [package] [filename.launch]
```

まず最初に、先ほど作成してビルドしたbeginner_tutorialsパッケージに移動します。

```
$ roscd beginner_tutorials
```

ここで、パッケージが見つからないというエラー（No such package/stack 'beginner_tutorials' , create_a_workspace）が表示された場合は、チュートリアルの最後に行ったように、環境設定ファイルをsourceにする必要があります。

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscd beginner_tutorials
```

そして、起動用のディレクトリを作りましょう。

```
$ mkdir launch
$ cd launch
```

注：起動ファイルを格納するディレクトリは、必ずしもlaunchという名前である必要はありません。roslaunchコマンドは、渡されたパッケージを自動的に調べて、利用可能な起動ファイルを検出します。しかし、これは良い習慣と考えられています。

### 起動ファイル

それでは、turtlemimic.launchという起動ファイルを作成し、以下のように貼り付けてみましょう。

```
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```

### 起動ファイルの説明

では、launchのxmlを分解してみましょう。
行番号表示/非表示切替

```
   1 <launch>
```

ここでは、launchタグで起動ファイルを開始し、起動ファイルであることがわかるようにしています。

```
   3   <group ns="turtlesim1">
   4     <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
   5   </group>
   6 
   7   <group ns="turtlesim2">
   8     <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
   9   </group>
```

ここでは、turtlesim1とturtlesim2という名前の名前空間タグを持つ2つのグループを、simという名前のturtlesimノードで開始しています。これにより、名前の衝突を起こすことなく、2つのシミュレータを起動することができます。

```
  11   <node pkg="turtlesim" name="mimic" type="mimic">
  12     <remap from="input" to="turtlesim1/turtle1"/>
  13     <remap from="output" to="turtlesim2/turtle1"/>
  14   </node>
```

ここでは、トピックのinputとoutputの名前をturtlesim1とturtlesim2に変更してmimicノードを開始します。この名前の変更により、turtlesim2がturtlesim1を模倣することになります。

```
  16 </launch>
```

起動ファイルのxmlタグを閉じます。

### roslaunching

それでは、起動ファイルをroslaunchしてみましょう。

```
$ roslaunch beginner_tutorials turtlemimic.launch
```

2台のturtleimicが起動し、新しいターミナルでrostopicコマンドを送ります。

```
$ rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

publishコマンドがturtlesim1にしか送られていないにもかかわらず、2つのturtlesimが動き出すのがわかります。

<img src="http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch?action=AttachFile&do=get&target=mimic.png">

また、rqt_graphを使ってローンチファイルの動作をよりよく理解することができます。rqtのメインウィンドウを起動し、「Plugins」→「Introspection」→「Node Graph」を選択します。

```
$ rqt
```

あるいは単に

```
$ rqt_graph
```

<img src="http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch?action=AttachFile&do=get&target=mimiclaunch.jpg">


rqt_consoleとroslaunchの使用に成功したところで、ROSのエディタオプションについて学びましょう。次のチュートリアルでは必要ありませんので、TurtlesimsはすべてCtrl-Cで停止してください。

Except where otherwise noted, the ROS wiki is licensed under the
[Creative Commons Attribution 3.0](http://creativecommons.org/licenses/by/3.0/)

Wiki: [ROS/Tutorials/UsingRqtconsoleRoslaunch](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch) (最終更新日時 2018-01-06 15:45:32 更新者 ErikNewton)
