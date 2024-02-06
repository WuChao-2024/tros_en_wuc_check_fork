# 1.2 apt installation and upgrade

This section introduces how to use apt to install TogetheROS.Bot on Horizon RDK and X86 platforms.

## Horizon RDK platform

Prerequisites

- The environment preparation in section 1.1 has been completed.
- The Horizon RDK system has been installed with version 2.x.
- The Horizon RDK can access the internet normally.
- The Horizon RDK can be accessed remotely via SSH.

**Note**

| Dependency  | 1.x tros.b  | 2.x tros.b |
| ----------- | ------------| ------------|
| 1.x system image |       √     |       x     |
| 2.x system image |       x     |       √     |

- **The 2.x version of tros.b only supports 2.x version of the system image, and the [1.x version of tros.b](https://developer.horizon.cc/api/v1/fileData/TogetherROS/index.html) only supports 1.x version of the system.**

- **If you are using a 1.x version of the system image, you need to [upgrade the system](./preparation) to the 2.x version.**

- **For information on how to check the system and tros.b version numbers and detailed instructions, please refer to the [FAQs](../FAQs/hardware_and_system.md).**

### Install tros.b

**Note: The IP address of the Horizon RDK used here is 10.64.61.241. Replace it with your own Horizon RDK IP address during installation.**

Login to the Horizon RDK

```shell
ssh root@10.64.61.241
```

Install the tros.b package, `sudo apt update; sudo apt install tros`

**Note: If you encounter the error `E: Unmet dependencies. Try 'apt --fix-broken install' with no packages (or specify a solution).' after running the installation command, please execute the command `apt --fix-broken install` to install the related dependencies before installing tros.b.**

After the installation is complete, check the files in the /opt directory

```bash
root@ubuntu:/userdata# ls /opt/
hobot  trosThe tros.b is installed in the /opt directory.

### Upgrade tros.b

Take Horizon RDK installation as an example, the X86 Ubuntu upgrade method is the same as Horizon RDK.

Login to Horizon RDK:

```shell
ssh root@10.64.61.241
```

Upgrade tros.b deb package:

```shell
sudo apt update
sudo apt upgrade
```

Check the current version of tros.b:

```bash
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

```

It can be seen that the current version of tros.b has been upgraded to version 2.0.0.

## X86 Platform

Prerequisites:

- The environment preparation work in Chapter 2.1 has been completed.
- The Ubuntu system is Ubuntu 20.04 and can access the Internet normally.

1. Set locale and enable the universe software repository:

   ```bash
   sudo apt update && sudo apt install localessudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
```

2. Download the GPG key file and add the source list:

```bash
sudo apt update && sudo apt install curl

sudo curl -sSL http://sunrise.horizon.cc/keys/sunrise.gpg -o /usr/share/keyrings/sunrise.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/sunrise.gpg] http://sunrise.horizon.cc/ubuntu-rdk-sim focal main" | sudo tee /etc/apt/sources.list.d/sunrise.list > /dev/null

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

3. Update the source information and install tros.b:

```bash
sudo apt update

sudo apt install tros
```

**Note:**

- **If you have already installed tros.b version 1.x on your X86 platform, please remove it with the command `sudo apt remove tros` before installing version 2.x of tros.b**.

- **To check the version number of tros.b, please refer to the [FAQs](../FAQs/hardware_and_system.md)**.