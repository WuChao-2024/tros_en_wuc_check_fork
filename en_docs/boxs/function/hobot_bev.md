---
sidebar_position: 10
---
# BEV Perception Algorithm

## Function Introduction

The BEV Perception Algorithm is a multi-task model trained on the [nuscenes](https://www.nuscenes.org/nuscenes) dataset using the Horizon [OpenExplorer](https://developer.horizon.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/bev.html) algorithm.

The algorithm takes 6 sets of image data as input, which are: frontal view, left front, right front, rear view, left rear, and right rear images. The model outputs 10 categories of objects and their corresponding 3D detection boxes, including obstacles, various types of vehicles, traffic signs, etc., as well as semantic segmentation of lane lines, sidewalks, and road edges.

This example uses local image data as input, performs algorithm inference using BPU, publishes image messages with rendered algorithm perception results, and renders and displays the algorithm results on the PC browser.

Code repository: <https://github.com/HorizonRDK/hobot_bev.git>

## Supported Platforms

| Platform  | Execution Method | Example Functionality            |
| --------- | --------------- | ------------------------------- |
| RDK Ultra | Ubuntu 20.04    | Local replay using web rendering |

## Preparation

1. RDK has been flashed to Horizon-provided Ubuntu 20.04 system image.

2. TogetheROS.Bot has been successfully installed on RDK.

3. Ensure that the PC can access RDK over the network.

## User Guide

### Local data replay

Performing local replay using local data, after inference, the algorithm perception results are published as image messages and rendered on the PC browser using the websocket package.

***Preparing the replay dataset***

```shell
# Download the dataset to the board
wget http://sunrise.horizon.cc/TogetheROS/data/hobot_bev_data.tar.gz

# Extract the dataset
mkdir -p hobot_bev_data
tar -zxvf hobot_bev_data.tar.gz -C hobot_bev_data

# After extraction, the dataset will be located at hobot_bev_data/data
```

***Replaying the dataset***# Configure tros.b environment
source /opt/tros/setup.bash

# Start websocket service
ros2 launch websocket websocket_service.launch.py

# Start running script and specify dataset path
ros2 launch hobot_bev hobot_bev.launch.py image_pre_path:=hobot_bev_data/data

## Result analysis

The terminal output during the execution is as follows:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-05-17-47-07-232907-hobot-2627970
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_bev-1]: process started with pid [2627972]
[INFO] [websocket-2]: process started with pid [2627974]
[hobot_bev-1] [WARN] [1688579227.907268364] [bev_node]:
[hobot_bev-1]  image_pre_path: hobot_bev_data/data
[hobot_bev-1] [BPU_PLAT]BPU Platform Version(1.3.3)!
[hobot_bev-1] [HBRT] set log level as 0. version = 3.14.25.0
[hobot_bev-1] [DNN] Runtime version = 1.12.3_(3.14.25 HBRT)
[hobot_bev-1] [WARN] [1688579228.714778531] [dnn]: Run default SetOutputParser.
[hobot_bev-1] [WARN] [1688579228.714925489] [dnn]: Set output parser with default dnn node parser, you will get all output tensors and should parse output_tensors in PostProcess.
[hobot_bev-1] [WARN] [1688579228.886846489] [bev_node]: loop 0/1002
[hobot_bev-1] [WARN] [1688579229.474568573] [bev_node]: loop 1/1002
[hobot_bev-1] [WARN] [1688579230.058551781] [bev_node]: loop 2/1002
[hobot_bev-1] [WARN] [1688579230.691667198] [bev_node]: loop 3/1002
[hobot_bev-1] [WARN] [1688579231.324658782] [bev_node]: loop 4/1002
[hobot_bev-1] [WARN] [1688579231.365145532] [bev_node]: input fps: 2.47, out fps: 2.52, infer time ms: 12, post process time ms: 659
[hobot_bev-1] [WARN] [1688579231.915645741] [bev_node]: loop 5/1002
[hobot_bev-1] [WARN] [1688579231.996993824] [bev_node]: input fps: 2.47, out fps: 2.52, infer time ms: 12, post process time ms: 658
```

You can view the image and algorithm rendering effects by entering http://IP:8000 in the browser on the PC (IP is the IP address of RDK):

![](./image/box_adv/render_bev.jpeg)