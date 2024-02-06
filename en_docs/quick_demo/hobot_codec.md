# 2.3 Image Codec

## Introduction

The image codec functionality is similar to the ROS image_transport package. The Horizon RDK utilizes hardware acceleration to convert between the MJPEG/H264/H265 and BGR8/RGB8/NV12 formats, which significantly reduces CPU usage while improving conversion efficiency. On the X86 platform, only the conversion between MJPEG and BGR8/RGB8/NV12 formats is supported.

Code repository: <https://github.com/HorizonRDK/hobot_codec>

## Supported Platforms

| Platform                       | Operating System | Example Functionality                                |
| ------------------------------ | ---------------- | --------------------------------------------------- |
| RDK X3, RDK X3 Module, RDK Ultra| Ubuntu 20.04     | Start MIPI camera to capture images, encode them, and display them via Web |
| X86                            | Ubuntu 20.04     | Publish YUV images using an image publishing tool, encode them, and display them via Web |

***RDK Ultra does not support H.264 video encoding.***

## Preparation

### Horizon RDK Platform

1. The Horizon RDK has been flashed with the Ubuntu 20.04 system image provided by Horizon.

2. TogetheROS.Bot has been successfully installed on the Horizon RDK.

3. The Horizon RDK has been connected to a camera, such as the F37 or other MIPI cameras.

### X86 Platform

1. The X86 environment has been configured with the Ubuntu 20.04 system image.

2. The X86 environment has been installed with the X86 version of tros.b.

## Usage

Taking JPEG encoding as an example, this section explains how to obtain NV12 format image data from a camera or image publishing tool, compress and encode it as JPEG, and preview the image on a PC via a web interface.

1. Obtain YUV data and start JPEG encoding:

    **Horizon RDK Platform**

    Log in to the Horizon RDK via SSH and use `mipi_cam` as the data source. Configure `hobot_codec` to input NV12 format and output JPEG format. Modify `mipi_cam` to the actual sensor model being used.

    a. Start `mipi_cam`

    ```shell
source /opt/tros/setup.bash

ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
```

b. Launch the hobot_codec encoder

```shell
source /opt/tros/setup.bash

ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
```

**X86 platform**

a. Launch the image publisher node

```shell
// Configure the tros.b environment:
source /opt/tros/setup.bash

// Copy the required image files for demonstration from the installation path of tros.b
cp -r /opt/tros/lib/hobot_image_publisher/config/ .

// Launch the image publisher node
ros2 launch hobot_image_publisher hobot_image_publisher.launch.py publish_output_image_w:=960 publish_output_image_h:=544 publish_message_topic_name:=/hbmem_img publish_fps:=20 
```

b. Launch the encoder & publisher package for JPEG images

```shell
source /opt/tros/setup.bash

ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
```

2. To view the JPEG encoded images on the web interface, open another terminal:

```shell
source /opt/tros/setup.bash
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
```

3. Open a web browser (Chrome/Firefox/Edge) on your PC and enter <http://IP:8000>. Replace IP with the IP address of the Horizon RDK/X86 device. Click on the Web端展示 (Web Display) at the top left to view the real-time JPEG encoded image.

 ![web-f37-codec](./image/hobot_codec/web-f37-codec.png "Real-time image")

## Important notes:If you encounter an abnormal startup of the Hobot codec node, you can troubleshoot the problem by following these steps:

1. Check if the tros.b environment is set.
2. Verify if the parameters are correct, please refer to the Hobot_codec README.md for details.