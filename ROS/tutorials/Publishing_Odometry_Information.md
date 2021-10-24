# Publishing Odometry Information over ROS

## 説明
このチュートリアルでは、ナビゲーションスタックのオドメトリ情報を公開する例を示します。ROS上でのnav_msgs/Odometryメッセージの発行と、tf上での "odom "座標フレームから "base_link "座標フレームへの変換の両方をカバーしています。

## ROS上でのオドメトリ情報の公開

ナビゲーションスタックでは、tfを使用してロボットの位置を決定し、センサーデータを静的な地図に関連付けることができます。しかし、tfはロボットの速度に関する情報を一切提供しません。このため、ナビゲーション・スタックは、オドメトリ・ソースがトランスフォームと、速度情報を含むnav_msgs/Odometryメッセージの両方をROS上でパブリッシュすることを要求しています。このチュートリアルでは、nav_msgs/Odometryメッセージについて説明し、メッセージとトランスフォームの両方をROSとtfでそれぞれパブリッシュするためのサンプルコードを提供します。

## nav_msgs/Odometry メッセージ

nav_msgs/Odometry メッセージは、自由空間におけるロボットの位置と速度の推定値を格納します。

```
# 自由空間における位置と速度の推定値を表しています。 
# このメッセージのposeはheader.frame_idで指定された座標フレームで指定する必要があります。
# このメッセージ中のツイストは，child_frame_id で指定された座標フレームで指定されるべきである．
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

このメッセージのposeは，Odometric frameにおけるロボットの推定位置と，そのpose推定値の確実性を示すオプションの共分散に対応する。
このメッセージに含まれるツイストは，子フレーム（通常はモバイルベースの座標フレーム）におけるロボットの速度に対応しており，その速度推定値の確実性を示すオプションの共分散も含まれています。

## Using tf to Publish an Odometry transform

Transform Configurationチュートリアルで説明したように、"tf "ソフトウェアライブラリは、トランスフォームツリー内のロボットに関連する座標フレーム間の関係を管理する役割を担っています。したがって、オドメトリソースは、それが管理する座標フレームに関する情報をパブリッシュする必要があります。以下のコードはtfの基本的な知識を前提としていますが、Transform Configurationチュートリアルを読めば十分でしょう。

## コードを書く

このセクションでは、ROS上でnav_msgs/Odometryメッセージを発行するサンプルコードと、ただ円を描いて走るだけの偽のロボットのためのtfを使ったトランスフォームを書いてみます。最初にコードの全体像を示し、以下では部分ごとに説明します。

パッケージの manifest.xml に依存関係を追加します。

```
<depend package="tf"/>
<depend package="nav_msgs"/>
```

```
   1 #include <ros/ros.h>
   2 #include <tf/transform_broadcaster.h>
   3 #include <nav_msgs/Odometry.h>
   4 
   5 int main(int argc, char** argv){
   6   ros::init(argc, argv, "odometry_publisher");
   7 
   8   ros::NodeHandle n;
   9   ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  10   tf::TransformBroadcaster odom_broadcaster;
  11 
  12   double x = 0.0;
  13   double y = 0.0;
  14   double th = 0.0;
  15 
  16   double vx = 0.1;
  17   double vy = -0.1;
  18   double vth = 0.1;
  19 
  20   ros::Time current_time, last_time;
  21   current_time = ros::Time::now();
  22   last_time = ros::Time::now();
  23 
  24   ros::Rate r(1.0);
  25   while(n.ok()){
  26 
  27     ros::spinOnce();               // check for incoming messages
  28     current_time = ros::Time::now();
  29 
  30     //compute odometry in a typical way given the velocities of the robot
  31     double dt = (current_time - last_time).toSec();
  32     double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  33     double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  34     double delta_th = vth * dt;
  35 
  36     x += delta_x;
  37     y += delta_y;
  38     th += delta_th;
  39 
  40     //since all odometry is 6DOF we'll need a quaternion created from yaw
  41     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  42 
  43     //first, we'll publish the transform over tf
  44     geometry_msgs::TransformStamped odom_trans;
  45     odom_trans.header.stamp = current_time;
  46     odom_trans.header.frame_id = "odom";
  47     odom_trans.child_frame_id = "base_link";
  48 
  49     odom_trans.transform.translation.x = x;
  50     odom_trans.transform.translation.y = y;
  51     odom_trans.transform.translation.z = 0.0;
  52     odom_trans.transform.rotation = odom_quat;
  53 
  54     //send the transform
  55     odom_broadcaster.sendTransform(odom_trans);
  56 
  57     //next, we'll publish the odometry message over ROS
  58     nav_msgs::Odometry odom;
  59     odom.header.stamp = current_time;
  60     odom.header.frame_id = "odom";
  61 
  62     //set the position
  63     odom.pose.pose.position.x = x;
  64     odom.pose.pose.position.y = y;
  65     odom.pose.pose.position.z = 0.0;
  66     odom.pose.pose.orientation = odom_quat;
  67 
  68     //set the velocity
  69     odom.child_frame_id = "base_link";
  70     odom.twist.twist.linear.x = vx;
  71     odom.twist.twist.linear.y = vy;
  72     odom.twist.twist.angular.z = vth;
  73 
  74     //publish the message
  75     odom_pub.publish(odom);
  76 
  77     last_time = current_time;
  78     r.sleep();
  79   }
  80 }
  ```

  さて、すべてに目を通したところで、コードの重要な部分を詳細に説明しましょう。

```
2 #include <tf/transform_broadcaster.h>
3 #include <nav_msgs/Odometry.h>
```

odom "座標フレームから "base_link "座標フレームへの変換と、nav_msgs/Odometryメッセージの両方を発行する予定なので、関連するヘッダーファイルをインクルードする必要があります。

```
9   ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
10   tf::TransformBroadcaster odom_broadcaster;
```

ROSとtfを使ってメッセージを送れるようにするには、ros::Publisherとtf::TransformBroadcasterの両方を作成する必要があります。

```
  12   double x = 0.0;
  13   double y = 0.0;
  14   double th = 0.0;
```

ここでは、ロボットが最初に「odom」座標フレームの原点からスタートしたと仮定します。

```
  16   double vx = 0.1;
  17   double vy = -0.1;
  18   double vth = 0.1;
```

ここでは、"base_link "フレームが "odom "フレームの中を、x方向に0.1m/s、y方向に-0.1m/s、th方向に0.1rad/sの割合で移動するような速度を設定します。これでほぼ、偽のロボットが円を描くように走行します。

```
 24   ros::Rate r(1.0);
```

この例では、イントロスペクションを容易にするために、1Hzのレートでオドメトリ情報を公開しますが、ほとんどのシステムではもっと高いレートでオドメトリを公開したいと思うでしょう。

```
  30     //compute odometry in a typical way given the velocities of the robot
  31     double dt = (current_time - last_time).toSec();
  32     double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  33     double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  34     double delta_th = vth * dt;
  35 
  36     x += delta_x;
  37     y += delta_y;
  38     th += delta_th;
```

ここでは、設定した一定の速度に基づいて、オドメトリ情報を更新します。もちろん、実際のオドメトリシステムでは、代わりに計算された速度を統合します。

```
  40     //since all odometry is 6DOF we'll need a quaternion created from yaw
  41     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
```

一般的に、システム内のすべてのメッセージに3Dバージョンを使用するようにしています。これは、2Dコンポーネントと3Dコンポーネントが必要に応じて連携できるようにするため、また、作成しなければならないメッセージの数を最小限に抑えるためです。そのため、オドメトリ用のヨー値をクオータニオンに変換して、ワイヤで送信する必要があります。幸いなことに、tfにはヨーの値からクオータニオンを簡単に作成したり、クオータニオンからヨーの値に簡単にアクセスできる関数が用意されています。

```
  43     //first, we'll publish the transform over tf
  44     geometry_msgs::TransformStamped odom_trans;
  45     odom_trans.header.stamp = current_time;
  46     odom_trans.header.frame_id = "odom";
  47     odom_trans.child_frame_id = "base_link";
```

ここでは、tf で送信する TransformStamped メッセージを作成します。ここでは、tf 経由で送信する TransformStamped メッセージを作成します。 current_time で "odom" フレームから "base_link" フレームにトランスフォームを発行したいと思います。したがって、メッセージのヘッダとchild_frame_idを適宜設定し、親座標フレームとして "odom "を、子座標フレームとして "base_link "を使用することを確認しています。

```
  49     odom_trans.transform.translation.x = x;
  50     odom_trans.transform.translation.y = y;
  51     odom_trans.transform.translation.z = 0.0;
  52     odom_trans.transform.rotation = odom_quat;
  53 
  54     //send the transform
  55     odom_broadcaster.sendTransform(odom_trans);
```

ここでは、オドメトリデータからトランスフォームメッセージを入力し、TransformBroadcasterを使用してトランスフォームを送信します。

```
  57     //next, we'll publish the odometry message over ROS
  58     nav_msgs::Odometry odom;
  59     odom.header.stamp = current_time;
  60     odom.header.frame_id = "odom";
```

また、nav_msgs/Odometryメッセージを発行して、ナビゲーション・スタックがそこから速度情報を得られるようにする必要があります。メッセージのヘッダには、current_timeと "odom "座標フレームを設定します。

```
  62     //set the position
  63     odom.pose.pose.position.x = x;
  64     odom.pose.pose.position.y = y;
  65     odom.pose.pose.position.z = 0.0;
  66     odom.pose.pose.orientation = odom_quat;
  67 
  68     //set the velocity
  69     odom.child_frame_id = "base_link";
  70     odom.twist.twist.linear.x = vx;
  71     odom.twist.twist.linear.y = vy;
  72     odom.twist.twist.angular.z = vth;
```

これにより、メッセージにオドメトリデータが入力され、ワイヤ経由で送信されます。メッセージのchild_frame_idを "base_link "フレームに設定します。"base_link "フレームは速度情報を送信する座標フレームだからです。
