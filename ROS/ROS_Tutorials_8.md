# ROSでのファイル編集にrosedを使用する

このチュートリアルでは、編集を容易にするためのrosedの使い方を説明します。

## rosedの使い方

rosed は rosbash スイートの一部です。パッケージへのパスをすべて入力するのではなく、パッケージ名を使ってパッケージ内のファイルを直接編集することができます。

使い方は以下の通りです。

```
$ rosed [package_name] [filename]
```

例を示します。

```
$ rosed roscpp Logger.msg
```

この例では、roscppパッケージ内のLogger.msgファイルを編集する方法を示しています。

この例がうまくいかない場合は、vimエディタがインストールされていないことが考えられます。エディターのセクションを参照してください。vimから抜け出す方法がわからない場合は、[ここ](http://kb.iu.edu/data/afcz.html)をクリックしてください。

ファイル名がパッケージ内で一意に定義されていない場合は、編集したいファイルを選択するメニューが表示されます。

## rosedをタブ補完で使う

この方法では、パッケージの正確な名前を知らなくても、パッケージ内のすべてのファイルを簡単に表示し、オプションで編集することができます。

使い方は以下の通りです。

```
$ rosed [package_name] <tab><tab>
```

例を示します。

```
$ rosed roscpp <tab><tab>
Empty.srv                   package.xml
GetLoggers.srv              roscpp-msg-extras.cmake
Logger.msg                  roscpp-msg-paths.cmake
SetLoggerLevel.srv          roscpp.cmake
genmsg_cpp.py               roscppConfig-version.cmake
gensrv_cpp.py               roscppConfig.cmake
msg_gen.py   
```

## エディタ

rosedのデフォルトのエディタはvimです。より初心者向けのエディタであるnanoは、Ubuntuのデフォルトインストールに含まれています。~/.bashrcファイルに以下のように記述することで使用できます。

```
export EDITOR='nano -w'
```

デフォルトのエディタを emacs にするには，~/.bashrc ファイルに次のように記述します。

```
export EDITOR='emacs -nw'
```

注意：.bashrc の変更は、新しいターミナルに対してのみ有効です。既に開いているターミナルには、新しい環境変数が表示されません。

新しいターミナルを開き，EDITORが定義されているかどうかを確認します。

```
$ echo $EDITOR
nano -w
```

または

```
$ echo $EDITOR
emacs -nw
```

rosedの設定と使用に成功したので、MsgとSrvを作成してみましょう。

Except where otherwise noted, the ROS wiki is licensed under the [Creative Commons Attribution 3.0](http://creativecommons.org/licenses/by/3.0/)

Wiki: [ROS/Tutorials/UsingRosEd](http://wiki.ros.org/ROS/Tutorials/UsingRosEd) (最終更新日時 2020-04-21 16:08:23 更新者 chapulina)