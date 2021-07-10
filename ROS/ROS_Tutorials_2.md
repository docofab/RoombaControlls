# ROSパッケージの作成



このチュートリアルでは、roscreate-pkgまたはcatkinを使用して新しいパッケージを作成し、rospackを使用してパッケージの依存関係をリストアップする方法を説明します。

## catkin パッケージとは何ですか？

パッケージがcatkinパッケージとみなされるためには、いくつかの要件を満たす必要があります。

* パッケージは、catkinに準拠したpackage.xmlファイルを含んでいなければなりません。
    * そのpackage.xmlファイルは、パッケージに関するメタ情報を提供します。
* パッケージには、catkin を使用する CMakeLists.txt が含まれている必要があります。
    * catkinメタパッケージの場合は、関連するボイラープレートのCMakeLists.txtファイルを持っていなければなりません。
* 各パッケージは独自のフォルダを持たなければなりません。
    * つまり、ネストされたパッケージや、複数のパッケージが同じディレクトリを共有することはありません。

最もシンプルなパッケージは、以下のような構造になっています。

```
my_package/
  CMakeLists.txt
  package.xml
```

## catkin ワークスペース内のパッケージ

catkinパッケージを扱う推奨方法は、catkinワークスペースを使用することですが、catkinパッケージをスタンドアロンで構築することもできます。一般的なワークスペースは以下のようになります。

```
workspace_folder/ -- ワークスペース
  src/ -- SOURCE SPACE
    CMakeLists.txt -- catkin が提供する「トップレベル」の CMake ファイル
    package_1/ -- パッケージ
      CMakeLists.txt -- package_1 用の CMakeLists.txt ファイル
      package.xml -- package_1用のパッケージマニフェスト
    ...
    package_n/
      CMakeLists.txt -- package_n用のCMakeLists.txtファイル
      package.xml -- package_nのパッケージマニフェスト
```

このチュートリアルを続ける前に、「Creating a workspace for catkin」のチュートリアルに従って、空の catkin ワークスペースを作成します。

## catkinパッケージの作成

このチュートリアルでは、catkin_create_pkg スクリプトを使って新しい catkin パッケージを作成する方法と、作成後にできることを説明します。

まず、Creating a Workspace for catkin チュートリアルで作成した catkin ワークスペースの source space ディレクトリに移動します。

```
# ワークスペース作成のチュートリアルで作成したはずです。
$ cd ~/catkin_ws/src
```

次に catkin_create_pkg スクリプトを使って、std_msgs, roscpp, rospy に依存する 'beginner_tutorials' という新しいパッケージを作成します。

```
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

これにより、beginner_tutorials フォルダが作成され、その中には package.xml と CMakeLists.txt が含まれ、catkin_create_pkg に与えた情報が部分的に記入されます。

catkin_create_pkg は、パッケージ名と、オプションとしてそのパッケージが依存する依存関係のリストを与えることを要求します。

```
# これは例であり、実行しようとしないでください。
# catkin_create_pkg <パッケージ名> [depend1] [depend2] [depend3].
```

catkin_create_pkg にはより高度な機能もあり、それは catkin/commands/catkin_create_pkg で説明されています。

## catkin ワークスペースの構築とセットアップファイルの調達

次に、catkin ワークスペース内のパッケージをビルドする必要があります。

```
$ cd ~/catkin_ws
$ catkin_make
```

ワークスペースがビルドされると、devel サブフォルダ内に通常の /opt/ros/$ROSDISTRO_NAME の下にあるのと同様の構造が作成されます。

ワークスペースをROS環境に追加するには、生成されたセットアップファイルをソースにする必要があります。

```
$ . ~/catkin_ws/devel/setup.bash
```

## パッケージの依存関係

### 一次の依存関係

以前にcatkin_create_pkgを使用したとき、いくつかのパッケージ依存関係が提供されていました。これらの一次依存関係を rospack ツールで確認できるようになりました。

```
$ rospack depends1 beginner_tutorials 
roscpp
rospy
std_msgs
```

ご覧のように、rospack は catkin_create_pkg を実行したときに引数として使用されたものと同じ依存関係をリストアップしています。これらのパッケージの依存関係は、package.xmlファイルに格納されています。

```
$ roscd beginner_tutorials
$ roscd beginner_tutorials $ cat package.xml

<package format="2">
...
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
...
</package>
```

### 間接的な依存関係

多くの場合、ある依存関係はそれ自身の依存関係も持ちます。例えば、rospyは他の依存関係を持っています。

```
$ rospack depends1 rospy
genpy
roscpp
rosgraph
rosgraph_msgs
roslib
STD_MSGS
```

パッケージはかなりの数の間接的な依存関係を持つことがあります。幸運なことに、rospackはすべての入れ子になった依存関係を再帰的に決定することができます。

```
$ rospack depends beginner_tutorials
cpp_common
rostime
roscpp_traits
roscpp_serialization
catkin
genmsg
genpy
message_runtime
gencpp
geneus
genodejs
GENLISP
message_generation
rosbuild
rosconsole
STD_MSGS
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
ros_environment
rospack
roslib
rospy
```

## パッケージのカスタマイズ

チュートリアルのこの部分では、catkin_create_pkgによって生成された各ファイルを見て、それらのファイルの各コンポーネントと、あなたのパッケージのためにそれらをどのようにカスタマイズできるかを、一行ずつ説明します。

### package.xmlのカスタマイズ

生成されたpackage.xmlは、新しいパッケージの中にあるはずです。それでは、新しいpackage.xmlを見て、注意が必要な要素を修正してみましょう。

#### 説明タグ

まず、descriptionタグを更新します。

```
   5  <description>The beginner_tutorials package</description>
```
説明文は自由に変更できますが、慣習的に、最初の文はパッケージの範囲をカバーしながら短くする必要があります。一文で説明するのが難しい場合は、パッケージを分割する必要があるかもしれません。

#### メンテナタグ

次にメンテナタグです。

```
   7 <!-- 1 つのメンテナタグが必要ですが、複数あっても構いませんし、1 つのタグにつき 1 人の担当者が必要です --> 
   8 <!-- 例: -->
   9 <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  10 <maintainer email="user@todo.todo">user</maintainer>
```

これは package.xml の必須かつ重要なタグで、パッケージについて誰に連絡すればよいかを他の人に知らせるためのものです。少なくとも 1 人のメンテナが必要ですが、必要に応じて多くのメンテナを置くこともできます。メンテナの名前はタグの本文に記述しますが、Eメール属性もありますので、記入してください。

```
   7 <maintainer email="you@yourdomain.tld">あなたの名前</maintainer>
```

#### ライセンスタグ

次にライセンスタグですが、これも必須です。

```
  12 <!-- 1つのライセンスタグが必要ですが、複数のライセンスタグを使用することができ、1つのタグにつき1つのライセンスが必要です。
  13 <!-- 一般的に使用されるライセンス文字列: -->
  14 <!-- BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  15 <license>TODO</license>
```

ライセンスを選んでここに記入します。一般的なオープンソース・ライセンスには、BSD、MIT、Boost Software License、GPLv2、GPLv3、LGPLv2.1、LGPLv3などがあります。これらのライセンスについては、The Open Source Initiativeをご覧ください。このチュートリアルでは、ROSの他のコアコンポーネントがすでにBSDライセンスを使用していることから、BSDライセンスを使用します。

```
   8 <license>BSD</license>
```

#### dependenciesタグ

次のタグ群は、あなたのパッケージの依存関係を記述します。依存関係は build_depend, buildtool_depend, exec_depend, test_depend に分けられます。これらのタグの詳細な説明については、Catkin Dependenciesに関するドキュメントを参照してください。catkin_create_pkgの引数にstd_msgs, roscpp, rospyを渡しているので、依存関係は以下のようになります。

```
  27 <!-- *_depend タグは、依存関係を指定するために使用されます -->。
  28 <!-- 依存関係には、catkinのパッケージやシステムの依存関係があります -->。
  29 <!--例 -->
  30 <!-- コンパイル時に必要なパッケージにはbuild_dependを使う： -->
  31 <!-- <build_depend>genmsg</build_depend> -->
  32 <!-- ビルドツールパッケージには buildtool_depend を使用します。-->
  33 <!-- <buildtool_depend>catkin</buildtool_depend> -->
  34 <!-- 実行時に必要なパッケージにはexec_dependを使う: -->
  35 <!-- <exec_depend>python-yaml</exec_depend> -->
  36 <!-- テスト時にのみ必要なパッケージには test_depend を使用してください: -->
  37 <!-- <test_depend>gtest</test_depend> -->
  38 <buildtool_depend>catkin</buildtool_depend>
  39 <build_depend>roscpp</build_depend>
  40 <build_depend>rospy</build_depend>
  41 <build_depend>std_msgs</b><b> <build_depend>
```

リストアップされたすべての依存関係は、catkin のデフォルトの buildtool_depend に加えて、私たちのために build_depend として追加されました。この場合、指定したすべての依存関係がビルド時や実行時に利用できるようにしたいので、それぞれに exec_depend タグも追加します。

```
  12   <buildtool_depend>catkin</buildtool_depend>
  13 
  14   <build_depend>roscpp</build_depend>
  15   <build_depend>rospy</build_depend>
  16   <build_depend>std_msgs</build_depend>
  17 
  18   <exec_depend>roscpp</exec_depend>
  19   <exec_depend>rospy</exec_depend>
  20   <exec_depend>std_msgs</exec_depend>```
```

#### 最終的なpackage.xml

ご覧のように、コメントや未使用のタグを除いた最終的な package.xml は、より簡潔なものになっています。

```
   1 <?xml version="1.0"?>
   2 <package format="2">
   3   <name>beginner_tutorials</name>
   4   <version>0.1.0</version>
   5   <description>The beginner_tutorials package</description>
   6 
   7   <maintainer email="you@yourdomain.tld">Your Name</maintainer>
   8   <license>BSD</license>
   9   <url type="website">http://wiki.ros.org/beginner_tutorials</url>
  10   <author email="you@yourdomain.tld">Jane Doe</author>
  11 
  12   <buildtool_depend>catkin</buildtool_depend>
  13 
  14   <build_depend>roscpp</build_depend>
  15   <build_depend>rospy</build_depend>
  16   <build_depend>std_msgs</build_depend>
  17 
  18   <exec_depend>roscpp</exec_depend>
  19   <exec_depend>rospy</exec_depend>
  20   <exec_depend>std_msgs</exec_depend>
  21 
  22 </package>
```

### CMakeLists.txtのカスタマイズ

メタ情報を含むpackage.xmlがパッケージに合わせて作成されたので、チュートリアルを進める準備ができました。catkin_create_pkgによって作成されたCMakeLists.txtファイルは、後のROSコードのビルドに関するチュートリアルで取り上げられます。

さて、新しいROSパッケージを作ったところで、ROSパッケージをビルドしてみましょう。


Reference sites: http://wiki.ros.org/ROS/Tutorials
Creative Commons Attribution 3.0
