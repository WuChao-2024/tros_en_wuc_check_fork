# Monocular 3D Indoor Detection

## Introduction

The mono3d_indoor_detection package is an example of indoor object 3D detection algorithm based on the hobot_dnn package. It uses the 3D detection model and indoor data on the Horizon's Horizon RDK to perform model inference using BPU and obtain the inference results.

Compared to 2D object detection, which can only recognize the object category and bounding box, 3D object detection can identify the precise position and orientation of the object. For example, in navigation the rich information provided by 3D object detection algorithms can help the planning and control robot achieve better effects.

The supported indoor object detection categories of the algorithm include: charging docks, trash cans, and slippers.

The detection results for each category include:

- Length, width, height: The length, width, and height of the three-dimensional object (i.e. hexahedron), measured in meters.

- Orientation: The orientation of the object relative to the camera, measured in radians. The range is from -π to π, representing the angle between the camera coordinate system x-axis and the object's forward direction in the camera coordinate system.

- Depth information: The distance from the camera to the object, measured in meters.

Code repository: <https://github.com/HorizonRDK/mono3d_indoor_detection>

Applications: The monocular 3D indoor detection algorithm can directly identify the exact position and orientation of objects in images, enabling object posture recognition. It is mainly used in autonomous driving, smart home, and other fields.

## Supported Platforms

| Platform              | System | Function                                       |
| --------------------- | ---------------- | ----------------------------------------------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04     | · Start MIPI/USB camera/local data and save the inference rendering result locally |
| X86                   | Ubuntu           | · Start local data offline and save the inference rendering result locally                  |

## Preparation

### Horizon RDK

1. Horizon RDK has been flashed with the Ubuntu 20.04 system image provided by Horizon.

2. TogetheROS.Bot has been successfully installed on the Horizon RDK.

### X86

1. X86 environment has been configured with Ubuntu 20.04 system image.

2. Tros.b has been successfully installed on the X86 environment.

## Usage

Because the 3D detection model is related to camera parameters, different cameras need to adjust the parameters accordingly.

The mono3d_indoor_detection algorithm package uses local image input for inference. After the inference, it can detect object categories and 3D positioning information, and publish the algorithm message for 3D information. Users can subscribe to the 3D detection result message for application development.

### Horizon RDK

```shell
# Configure tros environment
source /opt/tros/setup.bash

# Copy the required configuration files from the installation path of tros.b.
cp -r /opt/tros/lib/mono3d_indoor_detection/config/ .

# Launch the launch file
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 
```

### X86

```shell
# Configure ROS2 environment
source /opt/tros/setup.bash

# Copy the required configuration files from the installation path of tros.b.
cp -r /opt/tros/lib/mono3d_indoor_detection/config/ .

# Launch the launch file
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 
```

## Result Analysis

After processing one frame of image data, the mono3d_indoor_detection package outputs the following information:

```shell
[mono3d_indoor_detection-1] [INFO] [1662612553.868256257] [mono3d_detection]: target type: trash_can
[mono3d_indoor_detection-1] [INFO] [1662612553.868303755] [mono3d_detection]: target type: width, value: 0.236816
[mono3d_indoor_detection-1] [INFO] [1662612553.868358420] [mono3d_detection]: target type: height, value: 0.305664
[mono3d_indoor_detection-1] [INFO] [1662612553.868404002] [mono3d_detection]: target type: length, value: 0.224182
[mono3d_indoor_detection-1] [INFO] [1662612553.868448000] [mono3d_detection]: target type: rotation, value: -1.571989
[mono3d_indoor_detection-1] [INFO] [1662612553.868487790] [mono3d_detection]: target type: x, value: -0.191978
[mono3d_indoor_detection-1] [INFO] [1662612553.868530705] [mono3d_detection]: target type: y, value: -0.143963
[mono3d_indoor_detection-1] [INFO] [1662612553.868570870] [mono3d_detection]: target type: z, value: 0.714024
[mono3d_indoor_detection-1] [INFO] [1662612553.868611119] [mono3d_detection]: target type: depth, value: 0.714024
[mono3d_indoor_detection-1] [INFO] [1662612553.868651409] [mono3d_detection]: target type: score, value: 0.973215
[mono3d_indoor_detection-1] [INFO] [1662612553.868760238] [mono3d_detection]: target type: trash_can
[mono3d_indoor_detection-1] [INFO] [1662612553.868799486] [mono3d_detection]: target type: width, value: 0.253052
[mono3d_indoor_detection-1] [INFO] [1662612553.868842610] [mono3d_detection]: target type: height, value: 0.282349
[mono3d_indoor_detection-1] [INFO] [1662612553.868885191] [mono3d_detection]: target type: length, value: 0.257935
[mono3d_indoor_detection-1] [INFO] [1662612553.868929273] [mono3d_detection]: target type: rotation, value: -1.542728
[mono3d_indoor_detection-1] [INFO] [1662612553.868968855] [mono3d_detection]: target type: x, value: 0.552460
[mono3d_indoor_detection-1] [INFO] [1662612553.869010645] [mono3d_detection]: target type: y, value: -0.164073
[mono3d_indoor_detection-1] [INFO] [1662612553.869050018] [mono3d_detection]: target type: z, value: 1.088358
[mono3d_indoor_detection-1] [INFO] [1662612553.869088767] [mono3d_detection]: target type: depth, value: 1.088358
[mono3d_indoor_detection-1] [INFO] [1662612553.869126765] [mono3d_detection]: target type: score, value: 0.875521
```

The log displays the processing result of a frame. The result shows that the target type in the subscribed algorithm message is trash_can, and it also provides the 3D coordinates, distance, and rotation angle information of the trash_can.

The rendered result of a local image (which can be replaced by modifying the feed_image field in mono3d_indoor_detection.launch.py) is saved as an image in the result directory of the program. The corresponding inference result and rendering information of the image are as follows:

![](./image/box_adv/indoor_render.jpeg)