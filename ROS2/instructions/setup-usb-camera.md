# USBカメラ をROS2 Foxyで動かす手順

すでにワークスペース ~/create_ws ができている前提の手順です。  
以下のパッケージを使います。  
https://gitlab.com/boldhearts/ros2_v4l2_camera  
A ROS 2 camera driver using Video4Linux2 (V4L2).  

## ros2_v4l2_cameraのインストール

1. 以下の手順でros2_v4l2_cameraをインストールします。
    ```
    sudo apt install ros-foxy-v4l2-camera 
    ```

## v4l2_camera_nodeの起動

1. 以下の手順でノードを起動します。
    ```
    ros2 run v4l2_camera v4l2_camera_node
    ```
1. 問題無ければ /image_raw トピックが流れてきます。

## 画像データの確認

1. 画像ビューワーで確認します。
    ```
    ros2 run rqt_image_view rqt_image_view
    ```
1. Rviz2でもAdd→Image→Topic /image_rawを設定で表示されます。
