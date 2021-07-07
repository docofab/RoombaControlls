# ROS環境のインストールと設定

このチュートリアルでは、ROSのインストールと、コンピュータ上でのROS環境の設定について説明します。

## ROSのインストール

ROSのインストールは終わっているものとして進めます。

## 環境の管理

ROSのインストール中に、 いくつかあるsetup.*shファイルのうちの一つをsourceコマンドで実行するよう指示が出たり、このsourceコマンドの手順をシェルの起動スクリプトに追加するよう促されたりすることがあるでしょう。これは、ROSがシェル環境を使ってファイル空間を結合するという考えによるからです。これにより、異なるバージョンのROSや異なるパッケージのセットに対する開発が容易になります。

ROSのパッケージが見つからない、使えないという場合は、環境が正しく設定されているかどうかを確認してください。ROS_ROOTやROS_PACKAGE_PATHなどの環境変数が設定されているかどうかを確認するのが良い方法です。

```
$ printenv | grep ROS
```

もし環境変数が設定されていなければ、setup.*shファイルをsourceコマンドで実行する必要があるかもしれません。

環境設定ファイルは、さまざまな場所において提供されることがあります。

* パッケージマネージャでインストールされたROSパッケージはsetup.*shファイルを提供します。
* rosbuildのワークスペースは、roswsのようなツールを使ってsetup.*shファイルを提供します。
* setup.*shファイルは、catkinパッケージを構築またはインストールする際の副産物として作成されます。

注：チュートリアルの中では、rosbuildとcatkinについて言及しています。rosbuildはもう推奨されていませんし、メンテナンスもされていませんが、レガシーのために保管されています。 catkinはコードを整理するための推奨された方法で、より標準的なCMakeの規約を使用し、特に外部のコードベースを統合したい人や、自分のソフトウェアをリリースしたい人のために、より柔軟性を提供します。詳しい説明は catkin や rosbuild をご覧ください。

ROSをUbuntuのaptからインストールしたばかりなら、setup.*shファイルが「/opt/ros/<distro>/」にあるはずで、以下のようにソースを作成することができます。

```
$ source /opt/ros/<distro>/setup.bash
```

<distro>（「<>」を含む）は、ROSのディストリビューション名に置き換えてください（例：indigo、kinetic、lunarなど）。
ROS Kineticをインストールした場合は、次のようになります。

```
$ source /opt/ros/kinetic/setup.bash
```

この行を.bashrcに追加しない限り、ROSのコマンドにアクセスするためには、新しいシェルを開くたびにこのコマンドを実行する必要があります。このプロセスにより、同じコンピュータに複数のROSディストリビューション（例：indigoとkinetic）をインストールし、それらを切り替えることができます。

他のプラットフォームでは、ROSをインストールした場所に、このsetup.*shファイルがあります。

## ROS ワークスペースの作成

ここでは、ROS Groovy以降の手順を説明します。ROS Fuerte以前の場合は、rosbuildを選択します。

catkinのワークスペースを作成してビルドしてみましょう。

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

catkin_make コマンドは、catkin ワークスペースで作業するための便利なツールです。ワークスペースで最初に実行すると、'src'フォルダにCMakeLists.txtのリンクが作成されます。

ROS Melodic以前のPython 3ユーザーへの注意：
ROSをビルドしていて、システムを適切にセットアップしている（つまり、catkinなどの必要なROSのPythonパッケージのPython 3バージョンがインストールされている）場合、クリーンなcatkinワークスペースでの最初のcatkin_makeコマンドは次のようにしなければならないということです。

```
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

これで catkin_make が Python 3 で設定されます。以降のビルドには catkin_make だけを使うことができます。

さらに、カレントディレクトリを見ると、「build」と「devel」フォルダがあるはずです。「devel」フォルダの中には、いくつかのsetup.*shファイルがあるのがわかります。これらのファイルのいずれかをソースとすると、このワークスペースが環境の上にオーバーレイされます。これについて詳しく知りたい場合は、一般的な catkin のドキュメントを参照してください。続ける前に、新しい setup.*sh ファイルをsourceします。

```
$ source devel/setup.bash
```

ワークスペースがセットアップスクリプトによって適切にオーバーレイされることを確認するために、ROS_PACKAGE_PATH環境変数にあなたがいるディレクトリが含まれていることを確認します。

```
$ echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/kinetic/share
```

これで環境が整いましたので、ROSファイルシステムのチュートリアルを続けます。

# ROSのファイルシステムを操作する

## 前提条件

このチュートリアルでは、ros-tutorialsのパッケージを検査します。

```
$ sudo apt-get install ros-<distro>-ros-tutorials
```

<distro>（「<>」を含む）は、ROSのディストリビューション名に置き換えてください（例：indigo、kinetic、lunarなど）。

## ファイルシステムの概念の簡単な概要

パッケージ: パッケージは、ROSコードのソフトウェア構成単位です。各パッケージには、ライブラリ、実行ファイル、スクリプト、またはその他の成果物を含めることができます。

マニフェスト（package.xml）：マニフェストは、パッケージの説明です。マニフェストは、パッケージ間の依存関係を定義し、バージョン、メンテナ、ライセンスなどのパッケージに関するメタ情報を取得する役割を果たします。


## ファイルシステムツール

コードは多くのROSパッケージに分散しています。lsやcdなどのコマンドラインツールを使って移動するのは非常に面倒なので、ROSはそれを助けるツールを提供しています。

### rospackの使用

rospackはパッケージの情報を得ることができます。このチュートリアルでは、パッケージのパスを返すfindオプションについてのみ説明します。

使い方を説明します。

```
$ rospack find [パッケージ名]を入力します。
```

例

```
$ rospack find roscpp
```

は次のように返します。

```
YOUR_INSTALL_PATH/share/roscpp
```

Ubuntu LinuxでaptからROS Kineticをインストールした場合、正確には次のように表示されます。

```
/opt/ros/kinetic/share/roscpp
```

### roscdの使用

roscdは、rosbashスイートの一部です。パッケージやスタックに直接ディレクトリを変更（cd）することができます。

使い方は以下のとおりです。

```
$ roscd <パッケージまたはスタック>[/subdir].
```

roscppパッケージ・ディレクトリに変更されたことを確認するには、次の例を実行します。

```
$ roscd roscpp
```

Unixのコマンドpwdを使って、作業ディレクトリを表示してみましょう。

```
$ pwd
```

次のように表示されるはずです。

```
YOUR_INSTALL_PATH/share/roscpp
```

YOUR_INSTALL_PATH/share/roscppは、先ほどの例でrospack findが指定したパスと同じであることがわかります。

roscdは、他のROSツールと同様に、ROS_PACKAGE_PATHに記載されているディレクトリ内にあるROSパッケージしか見つけられないことに注意してください。ROS_PACKAGE_PATHに何があるかを確認するには、次のように入力します。

```
$ echo $ROS_PACKAGE_PATH
```

ROS_PACKAGE_PATHには、ROSのパッケージがあるディレクトリをコロンで区切ってリストアップします。典型的なROS_PACKAGE_PATHは次のようなものです。

```
/opt/ros/kinetic/base/install/share
```

他の環境パスと同様に、ROS_PACKAGE_PATHに追加のディレクトリを追加することができ、各パスはコロン「:」で区切られます。

### サブディレクトリ

roscdはパッケージやスタックのサブディレクトリに移動することもできます。

試してみましょう。

```
$ roscd roscpp/cmake
$ pwd
```

次のようになります。

```
YOUR_INSTALL_PATH/share/roscpp/cmakeが表示されます。
```

### roscd ログ

roscd logは、ROSがログファイルを保存するフォルダに移動します。ROSのプログラムをまだ何も実行していない場合は、まだ存在しないというエラーが出ますので注意してください。

何かROSのプログラムを実行したことがある場合は、次のようにしてみてください。

```
$ roscd log
```

## roslsの使用

roslsは、rosbashスイートの一部です。絶対パスではなく、名前でパッケージ内を直接lsすることができます。

使い方は以下の通りです。

```
$ rosls <パッケージまたはスタック>[/subdir].
```

例

```
$ rosls roscpp_tutorials
```

は次のように返します。

```
cmake launch package.xml srv
```

### タブ補完

パッケージの名前を全部入力するのは面倒ですよね。前述の例では、roscpp_tutorialsはかなり長い名前です。幸運なことに、いくつかのROSツールはTAB補完をサポートしています。

まずは入力してみましょう。

```
$ roscd roscpp_tut<<ここでTABキーを押してください>>>と入力します。
```

TABキーを押すと、あとはコマンドラインが補完してくれます。

```
$ roscd roscpp_tutorials/
```

roscpp_tutで始まるROSパッケージは現在のところroscpp_tutorialsだけなので、これでうまくいきます。

では、次のように入力してみてください。

```
$ roscd tur<<ここでTABキーを押してください>>。
```

TABキーを押した後、コマンドラインができるだけ埋まるようにします。

```
$ roscd turtle
```

しかし、この場合、turtleで始まるパッケージが複数あります。もう一度、TABキーを押してみてください。これでturtleで始まるROSパッケージがすべて表示されるはずです。

```
turtle_actionlib/ turtlesim/ turtle_tf/...
```

コマンドラインでは以下のようになっているはずです。

```
$ roscd turtle
```

次にturtleの後にsを入力してTABキーを押します。

```
$ roscd turtles<< now push the TAB key >>>.
```

turtlesで始まるパッケージは1つしかないので、次のようになります。

```
$ roscd turtlesim/
```

現在インストールされているすべてのパッケージのリストを見たい場合は、同様にタブ補完を使用できます。

```
$ rosls <<< 今度はTABキーを2回押してみてください >> >>
```

## まとめ

ROSツールのネーミングにはパターンがあることにお気づきでしょうか。

```
rospack = ros + pack(age)
roscd = ros + cd
rosls = ros + ls 
```

この命名パターンは、ROSツールの多くに当てはまります。

さて、ROSを使いこなせるようになったところで、パッケージを作ってみましょう。
