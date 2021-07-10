# ROSパッケージの構築

このチュートリアルでは、パッケージをビルドするためのツールチェーンについて説明します。

## パッケージの構築

パッケージのシステム依存関係がすべてインストールされていれば、新しいパッケージをビルドすることができます。

Note: aptやその他のパッケージマネージャを使ってROSをインストールした場合は、すでにすべての依存関係があるはずです。

続ける前に、環境設定ファイルのソースを確認してください。Ubuntuでは次のようになります。

```
# source /opt/ros/%YOUR_ROS_DISTRO%/setup.bash
$ source /opt/ros/kinetic/setup.bash # 例えばKineticの場合
```

catkin_makeの使い方

catkin_makeは、標準的なcatkinのワークフローにいくつかの利便性を追加するコマンドラインツールです。catkin_makeは、標準的なCMakeのワークフローにおけるcmakeとmakeの呼び出しを組み合わせたものだと想像できます。

使い方は以下の通りです。

```
# catkinのワークスペースで
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...] 。
```

標準的なCMakeのワークフローに慣れていない人のために説明すると、以下のように分解されます。

Note: 以下のコマンドを実行しても動作しません。これは CMake が一般的にどのように動作するかの一例です。

```
# CMake プロジェクトの場合
$ mkdir build
$ cd build
$ cmake ...
$ make
$ make install # (オプションで)
```

このプロセスはCMakeプロジェクトごとに実行されます。対照的に、catkinプロジェクトはワークスペースで一緒にビルドすることができます。ワークスペースでゼロから多数の catkin パッケージをビルドするには、以下の作業フローに従います。

```
# catkinのワークスペースで
$ catkin_make
$ catkin_make install # (optionally)
```

上記のコマンドは、srcフォルダにあるすべてのcatkinプロジェクトをビルドします。これは、REP128で設定された推奨事項に従っています。もし、ソースコードが別の場所、例えばmy_srcにある場合は、catkin_makeをこのように呼び出します。

注：以下のコマンドを実行しても、my_srcというディレクトリが存在しないため、動作しません。

```
# catkinのワークスペースで
$ catkin_make -source my_src
$ catkin_make install -source my_src # (optionally)
```

catkin_makeのより高度な使い方については、ドキュメントを参照してください: catkin/commands/catkin_make

## パッケージのビルド

このページを使って自分のコードをビルドする場合、CMakeLists.txtを修正する必要があるかもしれないので、後のチュートリアル(C++)/(Python)も見てみてください。

前のチュートリアル「Creating a Package」で、すでにcatkinのワークスペースとbeginner_tutorialsという新しいcatkinパッケージができているはずです。catkinのワークスペースに入って、srcフォルダを見てみましょう。

```
$ cd ~/catkin_ws/
$ ls src
beginner_tutorials/ CMakeLists.txt@
```

すると、前のチュートリアルで catkin_create_pkg で作成した beginner_tutorials というフォルダがあるのがわかると思います。catkin_makeを使って、このパッケージをビルドしてみましょう。

```
$ catkin_make
```

cmakeとmakeの出力がたくさん出てきますが、これは次のようなものです。

```
Base path: /home/user/catkin_ws
Source space: /home/user/catkin_ws/src
Build space: /home/user/catkin_ws/build
Devel space: /home/user/catkin_ws/devel
Install space: /home/user/catkin_ws/install
####
#### Running command: "cmake /home/user/catkin_ws/src
-DCATKIN_DEVEL_PREFIX=/home/user/catkin_ws/devel
-DCMAKE_INSTALL_PREFIX=/home/user/catkin_ws/install" in "/home/user/catkin_ws/build"
####
-- The C compiler identification is GNU 4.2.1
-- The CXX compiler identification is Clang 4.0.0
-- Checking whether C compiler has -isysroot
-- Checking whether C compiler has -isysroot - yes
-- Checking whether C compiler supports OSX deployment target flag
-- Checking whether C compiler supports OSX deployment target flag - yes
-- Check for working C compiler: /usr/bin/gcc
-- Check for working C compiler: /usr/bin/gcc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Using CATKIN_DEVEL_PREFIX: /tmp/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /opt/ros/kinetic
-- This workspace overlays: /opt/ros/kinetic
-- Found PythonInterp: /usr/bin/python (found version "2.7.1") 
-- Found PY_em: /usr/lib/python2.7/dist-packages/em.pyc
-- Found gtest: gtests will be built
-- catkin 0.5.51
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing packages in topological order:
-- ~~  - beginner_tutorials
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ add_subdirectory(beginner_tutorials)
-- Configuring done
-- Generating done
-- Build files have been written to: /home/user/catkin_ws/build
####
#### Running command: "make -j4" in "/home/user/catkin_ws/build"
####
```

catkin_makeでは、まず各「スペース」で使用しているパスを表示することに注意してください。スペースについては、REP128と、wikiのcatkin/workspacesのドキュメントで説明されています。重要なことは、これらのデフォルト値のために、あなたのcatkinワークスペースにいくつかのフォルダが作成されていることに気づくことです。lsで見てみましょう。

```
$ ls
build
devel
src
```

build フォルダーは、ビルド空間のデフォルトの場所で、パッケージの設定とビルドのために cmake と make が呼び出される場所です。develフォルダは、devel空間のデフォルトの場所で、パッケージをインストールする前に実行ファイルやライブラリが置かれる場所です。

さて、ROSパッケージをビルドしたところで、ROS Nodesについて詳しく説明しましょう。


Except where otherwise noted, the ROS wiki is licensed under the Creative Commons Attribution 3.0

Wiki: [ROS/Tutorials/BuildingPackages](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) (最終更新日時 2020-04-18 18:53:46 更新者 PedroAlcantara)
