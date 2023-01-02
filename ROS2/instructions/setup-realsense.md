# Intel RealSense を使う方法

## Resources

* [Intel RealSense for Developper](https://www.intelrealsense.com/developers/)
* [SDK 2.0](https://github.com/IntelRealSense/librealsense/releases)
* [Firmware update](https://dev.intelrealsense.com/docs/firmware-updates)

## ファームウェアのアップデート
Recommended Firmware  
D400 	5.14.0.0 or later

以下のものを用意する。
* Intel.RealSense.FW.Update.exe
* Signed_Image_UVC_5_14_0_0.bin

Windowsのコマンドラインから実行する。

```
Intel.RealSense.FW.Update.exe -f Signed_Image_UVC_5_14_0_0.bin
```

以下実行例。

```
C:\Users\XXXXX\Downloads\Signed_Image_UVC_5_14_0_0>dir
 ドライブ C のボリューム ラベルは Windows です
 ボリューム シリアル番号は 50FF-2255 です

 C:\Users\XXXXX\Downloads\Signed_Image_UVC_5_14_0_0 のディレクトリ

2022/12/30  18:38    <DIR>          .
2022/12/30  18:38    <DIR>          ..
2022/12/30  18:33        32,379,008 Intel.RealSense.FW.Update.exe
2022/12/30  18:38           411,101 RealSense-D400-Series-Spec-Update.pdf
2022/12/30  18:38         1,573,660 Signed_Image_UVC_5_14_0_0.bin
               3 個のファイル          34,363,769 バイト
               2 個のディレクトリ  247,698,829,312 バイトの空き領域

C:\Users\XXXXX\Downloads\Signed_Image_UVC_5_14_0_0>Intel.RealSense.FW.Update.exe

Nothing to do, run again with -h for help

Connected devices:
1) Name: Intel RealSense D435, serial number: 817412070782, update serial number: XXXXXXXXXXXX, firmware version: 05.13.00.50, USB type: 2.1

C:\Users\XXXXX\Downloads\Signed_Image_UVC_5_14_0_0>Intel.RealSense.FW.Update.exe -h

USAGE:

   Intel.RealSense.FW.Update.exe  [-b <string>] [-s <string>] [-f <string>]
                                  [-u] [-r] [-l] [--] [--version] [-h]


Where:

   -b <string>,  --backup <string>
     Create a backup to the camera flash and saved it to the given path

   -s <string>,  --serial_number <string>
     The serial number of the device to be update, this is mandetory if
     more than one device is connected

   -f <string>,  --file <string>
     Path of the firmware image file

   -u,  --unsigned
     Update unsigned firmware, available only for unlocked cameras

   -r,  --recover
     Recover all connected devices which are in recovery mode

   -l,  --list_devices
     List all available devices

   --,  --ignore_rest
     Ignores the rest of the labeled arguments following this flag.

   --version
     Displays version information and exits.

   -h,  --help
     Displays usage information and exits.


   librealsense rs-fw-update tool


C:\Users\XXXXX\Downloads\Signed_Image_UVC_5_14_0_0>Intel.RealSense.FW.Update.exe -f Signed_Image_UVC_5_14_0_0.bin

Warning! the camera is connected via USB 2 port, in case the process fails, connect the camera to a USB 3 port and try again

Updating device:
Name: Intel RealSense D435, serial number: 817412070782, update serial number: XXXXXXXXXXXX, firmware version: 05.13.00.50, USB type: 2.1
 30/12 18:39:48,088 WARNING [29424] (ds5-device.cpp:179) hr returned: HResult 0x8007001f: ....

Firmware update started

Firmware update progress: 100[%]

Firmware update done
 30/12 18:40:22,526 WARNING [10052] (enumerator-winusb.cpp:157) failed to locate usb interfaces for device: \\?\USB#VID_8086&PID_0ADC#111111111111#{a5dcbf10-6530-11d2-901f-00c04fb951ed

Device 817412070782 successfully updated to FW: 05.14.00.00

C:\Users\XXXXX\Downloads\Signed_Image_UVC_5_14_0_0>Intel.RealSense.FW.Update.exe

Nothing to do, run again with -h for help

Connected devices:
1) Name: Intel RealSense D435, serial number: 817412070782, update serial number: XXXXXXXXXXXX, firmware version: 05.14.00.00, USB type: 2.1

C:\Users\XXXXX\Downloads\Signed_Image_UVC_5_14_0_0>
```

## 動作確認 
以下のものを用意する。
* Intel.RealSense.Viewer.exe 

## Ubuntu 20.04へのlibrealsenseのインストール

以下のドキュメントに従う。  
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

* インストール後に、realsense-viewerが動作すればOK
* PCはパッケージインストールでOK
* Raspberry Pi 4はビルドが必要。(まだ未検証)

## realsense-rosのインストール

ROS2 Wrapper for Intel® RealSense™ Devices を使う。

以下のInstallation Instructionsに従う。  
https://github.com/IntelRealSense/realsense-ros

Step 1: Install the ROS2 distribution
Step 2: Install the latest Intel® RealSense™ SDK 2.0
Step 3: Install Intel® RealSense™ ROS2 wrapper from sources

### RealSenseのノードの起動方法

D435i でpointcloud, gyro, acclを有効にした場合

```
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true
```

### Rviz2での確認
RealSenseノードが起動できたらRviz2で状態を確認する。  
src/realsense-ros/realsense2_camera/launch/default.rvizがあるのでrviz2を起動して、File -> Open Configで読み込んでも良い。

主な設定は以下の通り
* Fixed Frameをcamera_depth_optical_frameにする。
* Add ImageでTopicを/camera/color/image_raw に設定
* Add ImageでTopicを/camera/depth/image_rect_rawに設定
* Add PointCloud2でTopicを/camera/depth/color/pointsに設定
