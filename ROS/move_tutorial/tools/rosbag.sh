#!/bin/sh

### @ref https://qiita.com/FluffyHernia/items/88d67195eb6c903ed942

### 使い方１　まずはターミナルでchmod +x dataprocess.sh で実行可能にする．
### 使い方２  実行するときは”./dataprocess.sh ~/処理したいrosbagのあるフォルダのパス/”
### 第一引数には処理したいbagファイルのありかを書こう！
### 指定したディレクトリ配下にあるbagファイルが表示されて正しいか確認したらyesのyを押そう！

# 第一引数で指定したディレクトリ配下のテキストファイルを一覧表示する。
echo "Files to be processed are..."
for file in `find $1 -name "*.bag"`; do
  echo " $file"
done


# 大丈夫？本当にこれらのbagfileで合ってるよね？っていう確認．
echo "Is it ok to continue? (type y/n)"
read INPUT

if [ "$INPUT" = "y" ]; then
  echo "Generating csv files..."

  for file in `find $1 -name "*.bag"`; do
    # パスの部分だけ
    path=$(dirname "${file}")
    # ファイル名を取り出す（拡張子あり）
    filename="${file##*/}"
    # 拡張子をなくす
    fname="${filename%.*}"
    #mkdir -p ${path}/csv_imu
    #mkdir -p ${path}/csv_position
    rostopic echo -b $file -p /create1/odom/pose/pose > "${path}csv_${fname}".csv
    #rostopic echo -b $file -p /position/pose > "${path}/csv_position/csv_${fname}".csv
  done

elif [ "$INPUT" = "n" ];then
  echo "Quit"
else
  echo "type y/n"
fi
