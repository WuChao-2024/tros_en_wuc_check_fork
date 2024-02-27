---
sidebar_position: 11
---

# Stereo Depth Estimation Algorithm

## Function Introduction

The Stereo Depth Estimation Algorithm is a `StereoNet` model trained using the Horizon [OpenExplorer](https://developer.horizon.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/stereonet.html) on the [SceneFlow](https://lmb.informatik.uni-freiburg.de/resources/datasets/SceneFlowDatasets.en.html) dataset.

The algorithm takes in stereo image data, namely left and right views, and outputs the disparity map of the left view.

This example uses the ZED 2i stereo camera as the input source, performs algorithm inference using BPU, publishes topic messages containing the left stereo image and the perception result, and renders and displays the algorithm result on a PC browser.

Code repository: <https://github.com/HorizonRDK/hobot_stereonet.git>

## Supported Platforms

| Platform  | Execution Method | Example Functions                      |
| --------- | --------------- | --------------------------------------- |
| RDK Ultra | Ubuntu 20.04    | Local playback and render result on web |

## Preparation

1. RDK has been burnt with Horizon-provided Ubuntu 20.04 system image.

2. TogetheROS.Bot has been successfully installed on RDK.

3. ZED 2i stereo camera is connected to the USB 3.0 interface of RDK.

4. Make sure the PC can access RDK through the network.

## Usage Instructions

Subscribe to the image data captured by the ZED 2i stereo camera as the input, publish topic messages containing the left stereo image and the perception result after inference, and render and display the published image and the corresponding algorithm result on a PC browser using the Websocket package.

Start command:

```shell
# Configure tros.b environment
source /opt/tros/setup.bash

# Start websocket service
ros2 launch websocket websocket_service.launch.py

ros2 launch hobot_stereonet hobot_stereonet_demo.launch.py 
```

## Result Analysis

```bash
The following information is outputted in the running terminal:[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-05-18-23-51-350999-hobot-2628272
[INFO] [launch]: Default logging verbosity is set to INFO
webserver has launch
[INFO] [hobot_stereo_usb_cam-1]: process started with pid [2628275]
[INFO] [talker-2]: process started with pid [2628277]
[INFO] [websocket-3]: process started with pid [2628279]
[INFO] [hobot_stereonet-4]: process started with pid [2628281]
[hobot_stereo_usb_cam-1] [WARN] [1688581432.042569331] [stereo_usb_cam_node]: Get params complete.
[hobot_stereo_usb_cam-1]  camera_name: default_cam
[hobot_stereo_usb_cam-1]  video_device index: 0
[hobot_stereo_usb_cam-1]  image_width: 1280
[hobot_stereo_usb_cam-1]  image_height: 720
[hobot_stereo_usb_cam-1]  io_method_name: shared_mem
[hobot_stereo_usb_cam-1]  pub_topic_name: hbmem_stereo_img
[hobot_stereo_usb_cam-1]  out_format: nv12
[hobot_stereo_usb_cam-1]  enable_fb: 0
[hobot_stereo_usb_cam-1]  enable_dump: 0
[hobot_stereonet-4] [WARN] [1688581432.071555206] [stereonet_node]:
[hobot_stereonet-4]  sub_hbmem_topic_name: hbmem_stereo_img
[hobot_stereonet-4]  ros_img_topic_name: /stereonet_node_output
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: ZED Open Capture - Camera module - Version: 0.6.0
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Camera resolution: 2560x720@30Hz
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Trying to open the device '/dev/video0'
[hobot_stereonet-4] [BPU_PLAT]BPU Platform Version(1.3.3)!
[hobot_stereonet-4] [HBRT] set log level as 0. version = 3.14.25.0
[hobot_stereonet-4] [DNN] Runtime version = 1.12.3_(3.14.25 HBRT)
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Opened camera with SN: 38085162
[hobot_stereo_usb_cam-1] [sl_oc::video::VideoCapture] INFO: Device '/dev/video0' opened
[hobot_stereonet-4] [WARN] [1688581432.344738873] [dnn]: Run default SetOutputParser.
[hobot_stereonet-4] [WARN] [1688581432.344880957] [dnn]: Set output parser with default dnn node parser, you will get all output tensors and should parse output_tensors in PostProcess.
[hobot_stereonet-4] [WARN] [1688581432.347218373] [stereonet_node]: model_input_count: 1, model_input_width: 1280, model_input_height: 720
[hobot_stereo_usb_cam-1] [WARN] [1688581432.412578248] [stereo_usb_cam_node]: Open video device 0 success.
[hobot_stereo_usb_cam-1] camera sn: 38085162[/dev/video0]
[hobot_stereonet-4] [WARN] [1688581434.992634291] [stereonet_node]: input fps: 1.60, out fps: 1.60, preprocess time ms: 1191, infer time ms: 48, msg preparation for pub time cost ms: 8
[hobot_stereonet-4] [WARN] [1688581436.203778417] [stereonet_node]: input fps: 0.82, out fps: 0.82, preprocess time ms: 1157, infer time ms: 47, msg preparation for pub time cost ms: 2
```
To view the image and algorithm rendering effects, enter http://IP:8000 in a PC browser (IP is the IP address of the RDK):

![stereonet_rdk](./image/box_adv/stereonet_rdk.png)

For the same scene, the depth estimation visualization of ZED is as follows:

![stereonet_zed](./image/box_adv/stereonet_zed.png)

It can be seen that for areas with changes in lighting, the accuracy of depth estimation is higher using the deep learning method.