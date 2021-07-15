# ROSのmsgとsrvの作成

このチュートリアルでは、msgおよびsrvファイルの作成とビルド方法、およびrosmsg、rosrv、roscpコマンドラインツールについて説明します。

## msg と srv の紹介

* msg: msg ファイルは、ROS メッセージのフィールドを記述したシンプルなテキス ト・ファイルです。さまざまな言語のメッセージのソースコードを生成するために使用されます。

* srv: srvファイルは、サービスを記述するファイルです。リクエストとレスポンスの2つの部分で構成されています。

msgファイルはパッケージのmsgディレクトリに、srvファイルはsrvディレクトリに格納されています。

msgsは、1行に1つのフィールドタイプとフィールド名を持つ単純なテキストファイルです。使用可能なフィールドタイプは以下のとおりです。

* int8, int16, int32, int64 (plus uint*)
* float32, float64
* string
* time, duration
* other msg files
* variable-length array[] and fixed-length array[C]

また、ROSにはHeaderという特殊な型があります。Headerには、ROSでよく使われるタイムスタンプや座標フレームの情報が含まれています。msgファイルの最初の行にHeaderヘッダーが付いているのをよく見かけます。

以下は、Header、文字列プリミティブ、および他の2つのmsgを使用するmsgの例です。

```
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

srvファイルは、msgファイルと似ていますが、リクエストとレスポンスの2つの部分で構成されています。2つの部分は「---」の行で区切られています。以下に、srvファイルの例を示します。

```
int64 A
int64 B
---
int64 Sum
```
上記の例では、A と B がリクエスト、Sum がレスポンスです。

## msgの使い方

### msgの作成

前回のチュートリアルで作成したパッケージに、新しいmsgを定義してみましょう。

```
$ roscd beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```

上の例の.msgファイルは、1行しか含まれていません。もちろん、次のように複数の要素を1行に1つずつ追加することで、より複雑なファイルを作ることができます。

```
string first_name
string last_name
uint8 age
uint32 score
```

しかし、もうひとつのステップがあります。msgファイルがC++やPythonなどのソースコードになっていることを確認する必要があります。

package.xmlを開いて、以下の2行が含まれていて、コメントされていないことを確認してください。

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

ビルド時には "message_generation"が必要ですが、ランタイム時には "message_runtime"だけが必要であることに注意してください。

CMakeLists.txtをお好みのテキストエディタで開きます（前のチュートリアルのrosedが良いでしょう）。

メッセージを生成できるように、CMakeLists.txtにすでに存在するfind_packageコールにmessage_generation依存関係を追加します。これは、単に message_generation を COMPONENTS のリストに追加するだけで、以下のような形になります。

```
# CMakeLists.txtに追加するだけではなく、既存のテキストを修正して、閉じ括弧の前にmessage_generationを追加してください。
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

ときどき、すべての依存関係でfind_packageを呼び出していなくても、プロジェクトがうまくビルドされることに気づくかもしれません。これは、catkinがすべてのプロジェクトを1つにまとめているためで、以前のプロジェクトがfind_packageを呼び出した場合、あなたのプロジェクトも同じ値で設定されます。しかし、この呼び出しを忘れると、あなたのプロジェクトが単独でビルドされたときに簡単に壊れてしまうことになります。

また、メッセージのランタイム依存関係をエクスポートしていることを確認してください。

```
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

次のコードブロックを見つけます。

```
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```

記号を削除してアンコメントし、Message*.msgファイルのスタンドをあなたの.msgファイルに置き換えて、次のようになります。

```
add_message_files(
  FILES
  Num.msg
)
```

手動で.msgファイルを追加することで、他の.msgファイルを追加した後に、CMakeがプロジェクトを再構成しなければならないタイミングを確実に知ることができます。

次に、generate_messages()関数が呼び出されていることを確認する必要があります。

ROS Hydro以降では、以下の行をアンコメントする必要があります。

```
# generate_messages()
#   DEPENDENCIES
#   std_msgs
# )
```

このようになります。

```
generate_messages()
  DEPENDENCIES
  std_msgs
)
```

以前のバージョンでは、1行をアンコメントするだけでいいかもしれません。

```
generate_messages()
```

これで、msg定義からソース・ファイルを生成する準備が整いました。今すぐに実行したい場合は、次のセクションを飛ばして、msgとsrvの共通ステップに進んでください。

## rosmsgの使用

以上で、msgの作成は完了です。それでは、rosmsg showコマンドを使って、ROSがmsgを見ることができるかどうか確認してみましょう。

使い方は以下の通りです。

```
$ rosmsg show [message type]
```

例を示します。

```
$ rosmsg show beginner_tutorials/Num
```

と表示されます。

```
int64 num
```

前述の例では、メッセージ・タイプは2つの部分から構成されています。

* beginner_tutorials -- メッセージが定義されているパッケージ
* Num -- msg Num の名前です。

メッセージがどのパッケージに入っているか覚えていない場合は、パッケージ名を省略することができます。試してみてください。

```
$ rosmsg show Num
```

次のように表示されます。

```
[beginner_tutorials/Num]:
int64 num
```

## srvの使用

### srvの作成

先ほど作成したパッケージを使って、srvを作成してみましょう。

```
$ roscd beginner_tutorials
$ mkdir srv
```

新しいsrv定義を手で作る代わりに、他のパッケージから既存のsrv定義をコピーします。

そのために、あるパッケージから別のパッケージにファイルをコピーするための便利なコマンドライン・ツールとして roscp があります。

使い方は

```
$ roscp [package_name] [file_to_copy_path] [copy_path]
```

それでは、rospy_tutorialsパッケージからサービスをコピーしてみましょう。

```
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

しかし、もうひとつのステップがあります。srvファイルがC++やPythonなどのソースコードになっていることを確認する必要があります。

すでに行っていなければ、package.xmlを開き、その中に以下の2行があり、コメントされていないことを確認してください。

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

前と同様に、ビルド時には "message_generation" が必要で、ランタイム時には "message_runtime" のみが必要であることに注意してください。

前のステップでメッセージについて既に行っていない限り、CMakeLists.txtにメッセージを生成するためのmessage_generation依存関係を追加します。

```
# CMakeLists.txtにこの行を追加するだけではなく、既存の行を修正してください。
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
```

(message_generationという名前にもかかわらず、msgとsrvの両方で動作します。)

また、サービス用のpackage.xmlもメッセージ用と同じように変更する必要がありますので、必要な追加の依存関係については上記をご覧ください。

以下の行のコメントを解除するために#を削除します。

```
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
```

そして、プレースホルダーのService*.srvファイルをあなたのサービスファイルに置き換えます。

```
add_service_files(
  FILES
  AddTwoInts.srv
)
```

これで、サービス定義からソースファイルを生成する準備が整いました。すぐに実行したい場合は、次のセクションを飛ばしてmsgとsrvの共通ステップに進んでください。

### rossrvの使用

srvを作成するために必要なことは以上です。rossrv showコマンドを使って、ROSがsrvを見られるようにしてみましょう。

使い方は以下の通りです。

```
$ rossrv show <service type>
```

例を示します。

```
$ rossrv show beginner_tutorials/AddTwoInts
```

以下のように表示されます。

```
    int64 a
    int64 b
    ---
    int64 sum
```

rosmsgと同様に、パッケージ名を指定せずにこのようなサービスファイルを見つけることができます。

```
$ rossrv show AddTwoInts
[beginner_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum

[rospy_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum
```

ここでは、2つのサービスが表示されています。1つ目はbeginner_tutorialsパッケージで作成したばかりのもので、2つ目はrospy_tutorialsパッケージであらかじめ用意されているものです。

### msgとsrvの共通手順

前のステップで既に行っていなければ、CMakeLists.txtを変更します。

```
# generate_messages(
#   DEPENDENCIES
# #  std_msgs  # Or other packages containing msgs
# )
```

これをアンコメントして、メッセージが使用する.msgファイルを含む依存しているパッケージ（ここではstd_msgs）を追加して、以下のようにします。

```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

さて、新しいメッセージを作ったので、パッケージを作り直す必要があります。

```
# あなたの catkin ワークスペースで
$ roscd beginner_tutorials
$ cd .../...
$ catkin_make
$ cd -
```

msg ディレクトリにある .msg ファイルは、サポートされているすべての言語で使用できるコードを生成します。C++メッセージのヘッダファイルは、\~/catkin_ws/devel/include/beginner_tutorials/に生成されます。Python スクリプトは \~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg に作成されます。lispファイルは、\~/catkin_ws/devel/share/common-lisp/ros/beginner_tutorials/msg/に表示されます。

同様に、srv ディレクトリにあるすべての .srv ファイルには、サポートされている言語のコードが生成されます。C++の場合は、メッセージヘッダファイルと同じディレクトリにヘッダファイルが生成されます。Python と Lisp の場合は、「msg」フォルダの隣に「srv」フォルダがあります。

メッセージ・フォーマットの完全な仕様は、メッセージ記述言語のページにあります。

新しいメッセージを使用する C++ノードを構築する場合は、catkin msg/srv build documentationで説明されているように、ノードとメッセージの間の依存関係を宣言する必要があります。

## ヘルプの取得

これまでにかなりの数のROSツールを見てきました。各コマンドがどのような引数を必要としているかを把握するのは難しいでしょう。幸い、ほとんどのROSツールは独自のヘルプを提供しています。

試してみてください。

```
$ rosmsg -h
```

    さまざまなrosmsgサブコマンドのリストが表示されるはずです。

```
      rosmsg show メッセージの説明を表示
      rosmsg list すべてのメッセージを表示する
      rosmsg md5 メッセージの md5sum を表示する
      rosmsg package パッケージ内のメッセージを一覧表示する
      rosmsg packages メッセージを含むパッケージを一覧表示する
```

サブコマンドのヘルプを表示することもできます。

```
$ rosmsg show -h
```

    rosmsg showに必要な引数を表示します。

```
    使用法: rosmsg show [オプション] <メッセージの種類

    オプション
      -h, --help このヘルプメッセージを表示して終了します。
      -r, --raw コメントを含む生のメッセージテキストを表示します。
```

## まとめ

これまでに使ったコマンドをいくつか挙げてみましょう。

* rospack = ros+pack(age) : ROSパッケージに関連する情報を提供します。
* roscd = ros+cd : ROSパッケージやスタックのディレクトリを変更する。
* rosls = ros+ls : ROSパッケージ内のファイルを一覧表示する
* roscp = ros+cp : ROSパッケージから/へファイルをコピーする
* rosmsg = ros+msg : ROSのメッセージ定義に関連する情報を提供する
* rossrv = ros+srv : ROSのサービス定義に関連する情報を提供する。
* catkin_make : ROSパッケージの作成(コンパイル)を行う。
    * rosmake = ros+make : ROSパッケージを作成(コンパイル)します(catkin workspaceを使用していない場合) 

## 次のチュートリアル

ROSの新しいmsgとsrvを作ったので、簡単なpublisherとsubscriber(python)(c++)の書き方を見てみましょう。



Except where otherwise noted, the ROS wiki is licensed under the [Creative Commons Attribution 3.0](http://creativecommons.org/licenses/by/3.0/)

Wiki: [ROS/Tutorials/CreatingMsgAndSrv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) (最終更新日時 2020-05-20 20:34:32 更新者 ShaneLoretz)

