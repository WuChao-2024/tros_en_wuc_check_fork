---
sidebar_position: 2
---
# 6.2 Applications and Examples

## Installation/Cross-compilation and Usage of Third-party Libraries on RDK X3

For cross-compilation and deployment, please refer to [Cross-Compilation Environment Setup](https://developer.horizon.cc/forumDetail/112555549341653662)

## How to Resolve "Compilation Process Killed" Error during Large Program Compilation?

Refer to [Swap Usage Tutorial](https://developer.horizon.cc/forumDetail/98129467158916281)

```shell
sudo mkdir -p /swapfile 
cd /swapfile 
sudo dd if=/dev/zero of=swap bs=1M count=1024 
sudo chmod 0600 swap 
sudo mkswap -f swap 
sudo swapon swap 
free

```

## How to Run the Camera Example?

Python provides examples for the FCOS algorithm based on F37 and GC4663 cameras, which will automatically detect the camera and perform algorithm inference.

```bash
cd /app/ai_inference/03_mipi_camera_sample
sudo python3 mipi_camera.py
```

Then, connect an HDMI monitor to render the images and display the algorithm results.

## Unable to View RGB888 RAW Images Published by RDK X3 Using rqt_image_view?

FastDDS does not implement MTU fragmentation at the UDP protocol layer, resulting in IP layer fragmentation. When UDP data is too large, routers and NICs cannot buffer a large number of fragments, and the loss of a single fragment will cause all fragments to be retransmitted, resulting in IP fragmentation attack. This leads to network congestion in the same subnet. To solve this, you can switch to cycloneDDS (command: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp), or send images with lower resolution in JPEG format to reduce the amount of data transmitted.

## Can Board Compilation be Supported on the Linux Image?

The Linux image rootfs has been minimized and does not support board-side compilation.

## How to Run Examples on the Linux Image?

The examples in the manual are introduced using the Ubuntu system as an example. The examples rely on Python for execution, and these examples can also be run on the RDK X3 with the Linux image (without Python).

- Instructions for Booting Ubuntu System and Linux Image Examples.Launching an example on Ubuntu system requires three steps:

1. Configure the `tros.b` environment by using the command `source /opt/tros/setup.bash`.

2. Copy the necessary configuration files to the execution path.

3. Launch the package in `tros.b` using either `ros2 run` or `ros2 launch`.

On a Linux image, these three steps are as follows:

1. Configure the `tros.b` environment by using the command `export LD_LIBRARY_PATH`.

2. Copy the necessary configuration files to the execution path.

3. Start the example program and specify the launch parameters. For packages written in C++, each package corresponds to an executable program.

Taking the algorithm reasoning example as an example, let's explain how to convert the launch script content into commands on a Linux image. The example takes a local image as input for reasoning, generates a rendered image, and saves it locally.

- Launch command on Ubuntu system:

The command is as follows:

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Copy the necessary configuration files needed for running the example from the installation path of tros.b. Config contains the model used by the example, and the local image used for the feedback.
cp -r /opt/tros/lib/dnn_node_example/config/ .

# Perform feedback prediction using a local jpg format image and save the rendered image
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py
```

- Launch script path:

This example uses `ros2 launch` to launch the `dnn_node_example package`. The launch script `dnn_node_example_feedback.launch.py` can be found in the installation path of `tros.b` on RDK X3: 

```shell
# find /opt/tros/ -name dnn_node_example_feedback.launch.py
/opt/tros/share/dnn_node_example/launch/dnn_node_example_feedback.launch.py
```

- Launch script content:

The main content of the launch script `dnn_node_example_feedback.launch.py` is as follows:

```python
def generate_launch_description():
    config_file_launch_arg = DeclareLaunchArgument(
        "dnn_example_config_file", default_value=TextSubstitution(text="config/fcosworkconfig.json")
```img_file_launch_arg = DeclareLaunchArgument(
        "dnn_example_image", default_value=TextSubstitution(text="config/test.jpg")
    )

    # Copy files in the config folder
    dnn_node_example_path = os.path.join(
        get_package_prefix('dnn_node_example'),
        "lib/dnn_node_example")
    print("dnn_node_example_path is ", dnn_node_example_path)
    cp_cmd = "cp -r " + dnn_node_example_path + "/config ."
    print("cp_cmd is ", cp_cmd)
    os.system(cp_cmd)

    return LaunchDescription([
        config_file_launch_arg,
        img_file_launch_arg,
        # Launch the package for single RGB human, head, face, hand bounding box and body keypoint detection
        Node(
            package='dnn_node_example',
            executable='example',
            output='screen',
            parameters=[
                {"feed_type": 0},
                {"config_file": LaunchConfiguration(
                    'dnn_example_config_file')},
                {"image": LaunchConfiguration('dnn_example_image')},
                {"image_type": 0},
                {"dump_render_img": 1}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
```


- Launch Script Description
The launch script supports selecting the algorithm to run through the configuration file specified by the parameter `dnn_example_config_file`, and the parameter `dnn_example_image` specifies the image used for algorithm inference. The example was not specified at startup, using the default configuration.

The package parameter in the launch script specifies that the package to be launched is named `dnn_node_example`,the `executable` specifies that the executable program is named `example`, and the `parameters` specifies the parameters to be passed to the executable program. For detailed instructions on using ROS2 launch, please refer to the ROS2 manual(http://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html).

- Package and executable program path

Find in the tros. b in path /opt/tros/of RDK X3:

```shell
# find /opt/tros/ -name dnn_node_example -type d
/opt/tros/lib/dnn_node_example
/opt/tros/share/dnn_node_example

# ls /opt/tros/lib/dnn_node_example
config  example
```

The executable program `example` can be found in the path `/opt/tros/lib/dnn_node_example`.

- Running the executable program on the Linux image

Execute the previously found executable program and include the `parameters` parameter from the launch script: `/opt/tros/lib/dnn_node_example/example --ros-args -p feed_type:=0 -p image_type:=0 -p dump_render_img:=1`.

- Running the example on the complete Linux image

```shell
# Configure the tros.b environment
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/tros/lib/

# Copy the configuration files needed to run the example from the tros.b installation path. The config directory contains the model used by example and the local images used for backtracking
cp -r /opt/tros/lib/dnn_node_example/config/ .

# Perform backtracking prediction using local jpg format images and store the rendered images
/opt/tros/lib/dnn_node_example/example --ros-args -p feed_type:=0 -p image_type:=0 -p dump_render_img:=1
```

## How to find the path of the launch script

The examples in the manual are launched using `ros2 launch`, for example, in the section **Boxs Algorithm Repository**, the subsection on **Toolchain Reference Algorithm** uses the launch script file `dnn_node_example.launch.py`. When modifications such as log level configurations are needed, it is necessary to first find the path of this script file in the `tros.b` installation path `/opt/tros/`.

To find the path of the launch script `dnn_node_example.launch.py`, use the following command:

```shell
# find /opt/tros/ -name dnn_node_example.launch.py
/opt/tros/share/dnn_node_example/launch/dnn_node_example.launch.py
```

## Slow speed when cross-compiling the TogetheROS.Bot source code

Due to the large number of packages in tros.b, compiling the source code takes some time (approximately 20 minutes on an 8-core CPU with 32GB of memory). There are two methods to speed up the process:

1. Minimal compilation

The compilation script provides two options: `all_build.sh` for complete compilation (the default compilation method in the cross-compilation section of the manual) and `minimal_build.sh` for minimal compilation. Minimal compilation does not compile algorithm examples and test cases, resulting in faster compilation speed.

To use minimal compilation, replace the command `./robot_dev_config/all_build.sh` with `./robot_dev_config/minimal_build.sh` in the configuration compilation options in the cross-compilation section of the manual.

2. Manually ignore unnecessary package compilation

In the package source code directory, create a file named `COLCON_IGNORE`. During compilation, this package will be ignored.

The downloaded package source code directory is specified in `robot_dev_config/ros2_release.repos`. For example, when downloading `google_benchmark_vendor`, configure it as follows:

```text
  ament/google_benchmark_vendor:
    type: git
    url: https://github.com/ament/google_benchmark_vendor.git
    version: 0.0.7
```

The code of the `google_benchmark_vendor` was downloaded in the path of `src/ament/google_benchmark_vendor`. Therefore, executing the command `touch src/ament/google_benchmark_vendor/COLCON_IGNORE` ignores the compilation of the `google_benchmark_vendor` package.

## Do you support installing and using other versions of ROS?

Yes, after installing tros.b on RDK X3, other versions of ROS, including ROS1, can also be installed and used.

:::caution **caution**
A terminal can only source one version of ROS. For example, after sourcing tros.b, you cannot source ROS2 Foxy or ROS1, or after sourcing ROS2 Foxy or ROS1, you cannot source tros.b again.
:::

In addition, the interface between tros.b and ROS2 foxy is fully compatible, and ROS2 rich toolkits can be reused without the need to install ROS2 foxy.

## Colcon compilation error
If the `colcon build` command is used to compile pkg, the following error is reported:

```shell
root@ubuntu:~/hobot_cam# colcon build
[4.933s] ERROR:colcon.colcon_core.package_identification:Exception in package identification extension 'ros' in 'hobot_cam': module 'pyparsing' has no attribute 'operatorPrecedence'
Traceback (most recent call last):
  File "/usr/lib/python3/dist-packages/catkin_pkg/condition.py", line 23, in evaluate_condition
    expr = _get_condition_expression()
  File "/usr/lib/python3/dist-packages/catkin_pkg/condition.py", line 44, in _get_condition_expression
    _condition_expression = pp.operatorPrecedence(
AttributeError: module 'pyparsing' has no attribute 'operatorPrecedence'
```

Perhaps the version of `python3-catkin-pkg` is lower and the condition function support is incomplete.


**Solution**

Upgrade the version of `python3-catkin-pkg` as follows:

```shell
# Add ROS apt source
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Delete old version
sudo apt remove python3-catkin-pkg

# Install old version
sudo apt update
sudo apt install python3-catkin-pkg
```

## How to check the tros.b version

After installing tros.b, log in to the system and use the command `apt show tros` to check the tros.b version.

For version 2.x (taking 2.0.0 version as an example), the tros.b information is as follows:

```shell
root@ubuntu:~# apt show tros
Package: tros
Version: 2.0.0-20230523223852
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: unknown
Depends: hobot-models-basic, tros-ros-base, tros-ai-msgs, tros-audio-control, tros-audio-msg, tros-audio-tracking, tros-body-tracking, tros-dnn-benchmark-example, tros-dnn-node, tros-dnn-node-example, tros-dnn-node-sample, tros-elevation-net, tros-gesture-control, tros-hand-gesture-detection, tros-hand-lmk-detection, tros-hbm-img-msgs, tros-hobot-app-xrrobot-body-tracking, tros-hobot-app-xrrobot-gesture-control, tros-hobot-codec, tros-hobot-cv, tros-hobot-falldown-detection, tros-hobot-hdmi, tros-hobot-image-publisher, tros-hobot-mot, tros-hobot-usb-cam, tros-image-subscribe-example, tros-img-msgs, tros-imu-sensor, tros-line-follower-model, tros-line-follower-perception, tros-mipi-cam, tros-mono2d-body-detection, tros-mono2d-trash-detection, tros-mono3d-indoor-detection, tros-parking-perception, tros-parking-search, tros-rgbd-sensor, tros-websocket, tros-xrrobot, tros-xrrobot-msgs
Download-Size: 980 B
APT-Manual-Installed: yes
APT-Sources: http://sunrise.horizon.cc/ubuntu-rdk focal/main arm64 Packages
Description: TogetheROS Bot

root@ubuntu:~#

```

For version 1.x (taking 1.1.6 version as an example), the tros.b information is as follows:

```shell
root@ubuntu:~# apt show tros
Package: tros
Version: 1.1.6
Section: utils
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: 1,536 MB
Pre-Depends: hhp-verify
Depends: symlinks, locales, hhp-verify, hobot-models-basic, hobot-arm64-libs (>= 1.1.6)
Apt-Sources: http://sunrise.horizon.cc/ubuntu-ports focal/main arm64 Packages
Date: 2023-03-24_17-29-12
Download-Size: 116 MB
APT-Manual-Installed: yes
Description: TogetherROS

N: There are 7 additional records. Please use the '-a' switch to see them.
root@ubuntu:~#

```

## Explanation of tros.b versions 1.x and 2.x

**Corresponding relationship with system versions and RDK platform hardware**

- tros.b version 2.x: Only supports system version 2.x; supports the entire series of RDK X3 hardware, such as RDK X3 and RDK X3 Module; future updates and new features of tros.b will be released in version 2.x; the code is hosted on GitHub.

- [tros.b version 1.x](https://developer.horizon.cc/api/v1/fileData/TogetherROS/index.html): Historical version; only supports system version 1.x and RDK X3; future updates and new features of tros.b version 1.x will only be issue fixes; the code is hosted on GitLab.

:::caution **Note**
Upgrading from tros.b version 1.x to 2.x cannot be done directly through the apt command. You need to reinstall the system using a burning image and then install tros.b version 2.x. [Here](https://developer.horizon.cc/documents_rdk/installation/install_os) are the instructions for installing the system.
:::

**Functional differences**

- Basic functionalities are the same. Future updates and new features of tros.b will only be based on version 2.x.

- Package management methods are different. tros.b version 1.x has only one installation package file, while tros.b version 2.x packages and releases installation packages separately based on functionality. Developers do not need to worry about the changes in package management methods.

**Usage differences**

- apt installation and upgrade, as well as source code compilation methods, remain the same (see the **System Installation** chapter for details).

- **Launch scripts for examples are different**. The file names and dependencies of the launch scripts have been optimized. Application examples should reference the launch scripts of dependent modules and configure the parameters accordingly. Please refer to this manual for the launch scripts of tros.b version 2.x examples.

## Failed to open the webpage in a web browser

Symptom: After entering the URL http://IP:8000 (IP is the IP address of RDK) in the web browser, the webpage fails to open. The possible reasons are as follows:

**nginx service is already running**

Cause: If the nginx service has already been started on RDK, for example, when running the web display example in RDK (without a port number, in which case entering http://IP address in the browser can open the webpage), starting the web display example of tros.b again will not start the nginx service, therefore specifying the port number will result in the webpage failing to open.

Solution: Kill the running nginx process on RDK or restart RDK.

## Only image is displayed in the web browser, no perception result rendering

1. Check if the rendering perception result feature is enabled in the web node startup command. For detailed parameter explanation, refer to the [README of hobot_websocket](https://github.com/HorizonRDK/hobot_websocket#%E5%8F%82%E6%95%B0).

2. Check if there are any error logs outputted in the web node startup terminal. If there are any, follow the prompt to troubleshoot.

3. Use the command `ros2 topic echo [topic name]` to confirm if perception result data is present.

4. Use the command `ps -x` to check if multiple web nodes are running. If there are any, use the `kill` command to stop all web node processes before restarting.