---
sidebar_position: 12
---
# Visual Inertial Odometry Algorithm

## Introduction

Visual Inertial Odometry (VIO) is an algorithm that combines camera and Inertial Measurement Unit (IMU) data to achieve robot localization. VIO positioning algorithm has the advantages of low cost and wide applicability. It can effectively compensate for the failure scenarios such as obstruction and multi-path interference in satellite positioning in outdoor environments. Excellent and robust VIO algorithm is the key to achieve high-precision outdoor navigation positioning.

![](./image/box_adv/hobot_vio_rviz.jpeg)

Code Repository: <https://github.com/HorizonRDK/hobot_vio.git>

## Supported Platforms

| Platform                       | Operating System | Example Function                                            |
| ------------------------------ | ---------------- | ------------------------------------------------------------ |
| RDK X3, RDK X3 Module, RDK Ultra | Ubuntu 20.04     | Use realsense camera images and IMU data as algorithm inputs; Algorithm outputs robot motion trajectory that can be visualized in rviz2 on PC. |

## Prerequisites

1. RDK has been flashed with the Ubuntu 20.04 system image provided by Horizon.

2. TogetheROS.Bot and Realsense ROS2 Package have been successfully installed on RDK.

3. Realsense camera is connected to the USB 3.0 interface of RDK.

4. Confirm that the PC can access RDK through the network.

## Usage Guide

The algorithm subscribes to the images and IMU data from the realsense camera as inputs. After processing, it calculates the trajectory information of the camera and publishes the camera's motion trajectory using the ROS2 topic mechanism. The trajectory result can be viewed in the rviz2 software on the PC. The input and output topics of the algorithm are shown in the table below:

### Input Topics

| Parameter    | Type         | Explanation                                                  | Required | Default Value                                      |
| ------------ | ------------ | ------------------------------------------------------------ | -------- | -------------------------------------------------- |
| path_config  | std::string  | Path to the VIO algorithm configuration file                 | Yes      | /opt/tros/lib/hobot_vio/config/realsenseD435i.yaml |
| image_topic  | std::string  | Image topic subscribed by the VIO algorithm                  | Yes      | /camera/infra1/image_rect_raw                      |
| imu_topic    | std::string  | IMU topic subscribed by the VIO algorithm                    | Yes      | /camera/imu                                        |
| sample_gap   | std::string  | VIO algorithm processing frequency: 1 means the trajectory calculation involves every frame image, 2 means every two frame images are calculated, and so on. | Yes      | 2                                                  |

### Output Topics

| Topic Name                   | Type                | Explanation                   |
| ---------------------------- | ------------------- | ------------------------------ |
| horizon_vio/horizon_vio_path | nav_msgs::msg::Path | Robot motion trajectory output by the VIO algorithm |

Launch command:

```shell
# Configure the tros.b environment
```source /opt/tros/setup.bash

ros2 launch hobot_vio hobot_vio.launch.py 
```

## 结果分析

在X3上启动算法示例后在运行终端输出如下信息，首先启动realsense节点发布图像和IMU数据，随后算法进入初始化流程，此时等待用户平移相机完成初始化，初始化完成后算法开始输出定位坐标：

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-07-19-48-31-464088-ubuntu-562910
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_vio-1]: process started with pid [563077]
[INFO] [ros2 launch realsense2_camera rs_launch.py  depth_module.profile:=640x480x30 enable_depth:=false enable_color:=false enable_gyro:=true enable_accel:=true enable_sync:=true gyro_fps:=200 accel_fps:=200 unite_imu_method:=2 enable_infra1:=true-2]: process started with pid [563081]
[hobot_vio-1] T_CtoI:
[hobot_vio-1]    0.999934   0.0103587   0.0049969   0.0270761
[hobot_vio-1]  -0.0104067    0.999899  0.00967935 -0.00272628
[hobot_vio-1] -0.00489613 -0.00973072    0.999941  -0.0518149
[hobot_vio-1]           0           0           0           1
[hobot_vio-1] system use_rtk_: 0
[hobot_vio-1] [static initializer] not enough imu readings
[hobot_vio-1] [static initializer] not enough imu readings
[hobot_vio-1] [static initializer] IMU belows th 0.011508, 0.00274453 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.0105996, 0.00273085 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00964632, 0.00280866 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00892132, 0.00279346 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00816016, 0.00281761 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00776753, 0.00277049 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00744219, 0.00274874 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.420251, 0.36058 < 0.5, 0
[hobot_vio-1] HorizonVIO Successfully initialized!
[hobot_vio-1] [WARN] [1688730518.534178615] [horizon_vio_node]: Localization position[x, y, z]: [0.0225533, -0.0504654, 0.00943574]
[hobot_vio-1] [WARN] [1688730518.534634139] [horizon_vio_node]: Image time 1688730518.314490318
[hobot_vio-1] [WARN] [1688730518.621440869] [horizon_vio_node]: Localization position[x, y, z]: [0.0231779, -0.0533648, 0.00787081]
[hobot_vio-1] [WARN] [1688730518.621558739] [horizon_vio_node]: Image time 1688730518.380982161
[hobot_vio-1] [WARN] [1688730518.743525086] [horizon_vio_node]: Localization position[x, y, z]: [0.0290396, -0.0610474, 0.0106718]
[hobot_vio-1] [WARN] [1688730518.743637249] [horizon_vio_node]: Image time 1688730518.447472572
[hobot_vio-1] [WARN] [1688730518.866076119] [horizon_vio_node]: Localization position[x, y, z]: [0.0381324, -0.0737757, 0.0164843]
[hobot_vio-1] [WARN] [1688730518.866186156] [horizon_vio_node]: Image time 1688730518.513962030
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 338
[hobot_vio-1] 132.853 ms all consumed
[hobot_vio-1] travel(m): 0.000
[hobot_vio-1] [WARN] [1688730519.002002975] [horizon_vio_node]: Localization position[x, y, z]: [0.05018, -0.088422, 0.0240244]
[hobot_vio-1] [WARN] [1688730519.002130095] [horizon_vio_node]: Image time 1688730518.580449104
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 31
[hobot_vio-1] 142.996 ms all consumed
[hobot_vio-1] travel(m): 0.014
[hobot_vio-1] [WARN] [1688730519.146149433] [horizon_vio_node]: Localization position[x, y, z]: [0.0167176, -0.0189649, 0.0588413]
[hobot_vio-1] [WARN] [1688730519.146279428] [horizon_vio_node]: Image time 1688730518.646935701
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 26
[hobot_vio-1] 96.911 ms all consumed
[hobot_vio-1] travel(m): 0.025
[hobot_vio-1] [WARN] [1688730519.244168068] [horizon_vio_node]: Localization position[x, y, z]: [0.000805884, 0.0134815, 0.0730707]
[hobot_vio-1] [WARN] [1688730519.244270439] [horizon_vio_node]: Image time 1688730518.713421583
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 23
[hobot_vio-1] 52.470 ms all consumed
[hobot_vio-1] travel(m): 0.034
[hobot_vio-1] [WARN] [1688730519.297642444] [horizon_vio_node]: Localization position[x, y, z]: [0.00226324, 0.0120054, 0.0796328]
[hobot_vio-1] [WARN] [1688730519.297738190] [horizon_vio_node]: Image time 1688730518.779906034
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 33
[hobot_vio-1] 47.407 ms all consumed
[hobot_vio-1] travel(m): 0.042