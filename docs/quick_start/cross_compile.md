---
sidebar_position: 3
---
# 1.3 Source Code Installation

This section explains how to install TogetheROS.Bot on the Horizon RDK and X86 platforms using source code.

## Horizon RDK Platform

Prerequisites:

- The development machine can access the Horizon Robotics organization on [GitHub](https://github.com/HorizonRDK).
- Docker is installed on the development machine.

### Compile

#### 1 Load docker image

All the following operations are performed within the Docker environment on the development machine.

```shell
## Create a directory
cd /mnt/data/kairui.wang/test
mkdir -p cc_ws/tros_ws/src
## Obtain the Docker for cross-compilation
wget http://sunrise.horizon.cc/TogetheROS/cross_compile_docker/pc_tros_v1.0.5.tar.gz
## Load the Docker image
sudo docker load --input pc_tros_v1.0.5.tar.gz
## Check the corresponding image ID for pc_tros
sudo docker images
## Launch Docker and mount the directory
sudo docker run -it --entrypoint="/bin/bash" -v PC local directory: Docker directory imageID, 
here is an example using:
sudo docker run -it --entrypoint="/bin/bash" -v /mnt/data/kairui.wang/test:/mnt/test 9c2ca340973e
```

#### 2 Obtain the Code

All the following operations are performed within the Docker environment on the development machine.

Here, we take the /mnt/test directory in Docker as an example.

```shell
cd /mnt/test/cc_ws/tros_ws
## Obtain the configuration file
git clone https://github.com/HorizonRDK/robot_dev_config.git -b develop
## Execute cd robot_dev_config and use the "git tag --list" command to view the available release versions
## Use the "git reset --hard [tag number]" command to specify the release version. For detailed instructions, refer to the "Compile Specific Version tros.b" section on this page
## Pull the source code
vcs-import src < ./robot_dev_config/ros2_release.repos
```
The directory structure of the entire project is as follows:

```text
├── cc_ws
│   ├── sysroot_docker
│   │   ├── etc
│   │   ├── lib -> usr/lib
│   │   ├── opt
│   │   └── usr
│   └── tros_ws
│       ├── robot_dev_config
│       └── src
```

The `tros_ws/robot_dev_config` path contains the configuration and script files needed for code fetching, compilation, and packaging. The `tros_ws/src` path stores the fetched code. The `sysroot_docker` path contains the header files and libraries required for cross-compilation, corresponding to the `/` directory of the Horizon RDK. For example, the path for the media library in `sysroot_docker` is `sysroot_docker/usr/lib/hbmedia/`, while the path in the Horizon RDK is `/usr/lib/hbmedia/`.

During compilation, the installation path of `sysroot_docker` is specified through the `CMAKE_SYSROOT` macro in the `robot_dev_config/aarch64_toolchainfile.cmake` compilation script.

#### 3 Cross-Compilation

All of these operations are performed inside the docker on the development machine.

```shell
## Compile tros.b version X3 using build.sh
bash ./robot_dev_config/build.sh -p X3

## Compile tros.b version RDK Ultra using build.sh
bash ./robot_dev_config/build.sh -p Rdkultra
```

After successful compilation, a message will prompt: N packages compiled and passed.

If using minimal_build.sh for minimal compilation, you can further compress the deployment package size by executing `./minimal_deploy.sh -d "install_path"`.

### Install

Copy the compiled directory to the Horizon RDK and rename it as tros. Here, we place the deployment package in the /opt/tros directory to be consistent with the deb installation directory.

### Compile a specific version

In the section **Compile**, in the step 2 **Obtain the Code**, the default is to fetch the latest version of tros.b source code. If you need to get a specific release version of the source code, you need to make the following modifications:

```bash
## Get the configuration file
git clone https://github.com/HorizonRDK/robot_dev_config.git -b develop 
cd robot_dev_config
## View available release versions
git tag --list
## Switch to the specified version number, here we take tros.b 2.0.0 as an example
git reset --hard tros_2.0.0
cd ..
## Pull code
vcs-import src < ./robot_dev_config/ros2_release.repos
```

## X86

### System Requirements

Required: Ubuntu 20.04 64-bit system. Alternatively, you can use X3 platform cross-compiling Docker images, but both compilation and execution must be performed within Docker.

### Configuration

#### Set local

Ensure that the language environment supports UTF-8.

```shell
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

#### Add apt sources

```shell
# First, make sure Ubuntu Universe is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl

# Add ROS2 official source
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add tros.b official source
sudo curl -sSL http://sunrise.horizon.cc/keys/sunrise.gpg -o /usr/share/keyrings/sunrise.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/sunrise.gpg] http://sunrise.horizon.cc/ubuntu-rdk-sim focal main" | sudo    tee /etc/apt/sources.list.d/sunrise.list > /dev/null
```

#### Install ROS tool packages

```shell
sudo apt update && sudo apt install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
```

### Obtain the Code

```shell
git config --global credential.helper store

mkdir -p ~/cc_ws/tros_ws/src
cd ~/cc_ws/tros_ws/

git clone https://github.com/HorizonRDK/robot_dev_config.git -b develop 
vcs-import src < ./robot_dev_config/ros2_release.repos
```

### Install Dependencies

Install the necessary packages for code compilation

```shell
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev

# install tros.b basic models
sudo apt install --no-install-recommends -y \hobot-models-basic

# install other packages dependencies
sudo apt install --no-install-recommends -y \
  qt5-qmake \
  libpyside2-dev \
  libshiboken2-dev \
  pyqt5-dev \
  python3-pyqt5 \
  python3-pyqt5.qtsvg \
  python3-pyside2.qtsvg \
  python3-sip-dev \
  shiboken2 \
  libyaml-dev \
  qtbase5-dev \
  libzstd-dev \
  libeigen3-dev \
  libxml2-utils \
  libtinyxml-dev \
  libssl-dev \
  python3-numpy \
  libconsole-bridge-dev \
  pydocstyle \
  libqt5core5a \
  libqt5gui5 \
  libgtest-dev \
  cppcheck \
  tango-icon-theme \
  libqt5opengl5 \
  libqt5widgets5 \
  python3-lark \
  libspdlog-dev \
  google-mock \
  clang-format \
  python3-flake8 \
  libbenchmark-dev \
  python3-pygraphviz \
  python3-pydot \
  python3-psutil \
  libfreetype6-dev \
  libx11-dev \
  libxaw7-dev \
  libxrandr-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  python3-pytest-mock \
  python3-mypy \
  default-jdk \
  libcunit1-dev \
  libopencv-dev \python3-ifcfg \
  python3-matplotlib \
  graphviz \
  uncrustify \
  python3-lxml \
  libcppunit-dev \
  libcurl4-openssl-dev \
  python3-mock \
  python3-nose \
  libsqlite3-dev \
  pyflakes3 \
  clang-tidy \
  python3-lttng \
  liblog4cxx-dev \
  python3-babeltrace \
  python3-pycodestyle \
  libassimp-dev \
  libboost-dev \
  libboost-python-dev \
  python3-opencv \
  libboost-python1.71.0
```

### Compilation

```shell
# Compile using build.sh
bash ./robot_dev_config/build.sh -p X86
```

After successful compilation, there will be an output showing N packages compiled.

### Installation

Copy the install directory generated during compilation to /opt directory and rename it as tros, in accordance with the deb installation directory.

## FAQ

Q1: How to determine if VCS successfully pulled the code?

A1: As shown in the image below, during the vcs import process, a "." indicates a successful repo pull, and an "E" indicates a failed repo pull. Specific failed repos can be seen in the log after execution. If this happens, you can try deleting the contents in the src directory and re-run vcs import or manually pull the failed repos.

![vcs_import](./image/cross_compile/vcs_import_error.png "vcs_import")

Q2: Limited conditions prevent code retrieval from GitHub

A2: You can directly download the desired version of the code from the [TogetheROS File Server](http://sunrise.horizon.cc/TogetheROS/source_code/). For example, the `tros_2.0.0_source_code.tar.gz` file corresponds to version 2.0.0 of tros.b.