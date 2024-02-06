# Hand Keypoint Detection

## Introduction

The hand keypoint detection algorithm example subscribes to images and intelligent messages containing hand bounding box information. It uses BPU for algorithm inference and publishes algorithm messages containing hand keypoint information.

The index of hand keypoints is shown in the following figure:

![](./image/box_adv/hand_lmk_index.jpeg)

Code repository:

<https://github.com/HorizonRDK/hand_lmk_detection>

<https://github.com/HorizonRDK/mono2d_body_detection>

Application scenarios: The hand keypoint detection algorithm is mainly used to capture skeletal keypoints of the hand, enabling functions such as custom gesture recognition. It is mainly applied in areas such as smart homes, virtual reality, and gaming entertainment.

## Supported Platforms

| Platform                        | Running Mode | Example Functionality                           |
| ------------------------------- | ------------ | ----------------------------------------------- |
| RDK X3, RDK X3 Module, RDK Ultra | Ubuntu 20.04 | · Start MIPI/USB camera and display inference results on the web |

## Preparation

### Horizon RDK Platform

1. The Horizon RDK has been flashed with the Ubuntu 20.04 system image provided by Horizon.

2. TogetheROS.Bot has been successfully installed on the Horizon RDK.

3. MIPI or USB cameras have been installed on the Horizon RDK.

4. Confirm that the PC can access the Horizon RDK through the network.

## Usage Instructions

The hand keypoint detection (hand_lmk_detection) package subscribes to images published by the sensor package and hand bounding box detection results published by the human body detection and tracking package. After inference, it publishes algorithm messages. The websocket package is used to render and display the published images and corresponding algorithm results on a PC browser.

**Publishing Images from a MIPI Camera**

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Copy the required configuration files from the installation path of tros.b
```cp -r /opt/tros/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/lib/hand_lmk_detection/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

**Publish pictures using a USB camera**

```shell
# Configure tros.b environment
source /opt/tros/setup.bash

# Copy the required configuration files for the example from the installation path of tros.b.
cp -r /opt/tros/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/lib/hand_lmk_detection/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

## Result Analysis

The terminal output during execution is as follows:

```shell
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_euclid_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_euclid_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[hand_lmk_detection-4] [WARN] [1660269063.553205182] [hand_lmk_det]: input fps: 31.43, out fps: 31.47
[hand_lmk_detection-4] [WARN] [1660269064.579457516] [hand_lmk_det]: input fps: 30.21, out fps: 30.21[hand_lmk_detection-4] [WARN] [1660269065.612579058] [hand_lmk_det]: input fps: 30.01, out fps: 30.01
[hand_lmk_detection-4] [WARN] [1660269066.612778892] [hand_lmk_det]: input fps: 30.00, out fps: 30.00
[hand_lmk_detection-4] [WARN] [1660269067.646101309] [hand_lmk_det]: input fps: 30.01, out fps: 30.01
[hand_lmk_detection-4] [WARN] [1660269068.679036184] [hand_lmk_det]: input fps: 30.04, out fps: 30.04

The output log shows that the program runs successfully and the input and output frame rates of the algorithm are 30fps, refreshing the statistical frame rate once per second.

On the PC browser, enter http://IP:8000 to view the image and algorithm rendering effect (where IP is the IP address of the Horizon RDK):

![](./image/box_adv/hand_render.jpeg)