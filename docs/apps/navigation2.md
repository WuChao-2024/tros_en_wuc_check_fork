---
sidebar_position: 2
---

# 4.2 Navigation2

## Introduction

Nav2 (Navigation2) is a built-in navigation framework in ROS2, aimed at finding a safe way for a mobile robot to move from point A to point B. Nav2 can also be applied to other robot navigation applications, such as dynamic point tracking, which requires dynamic path planning, motor speed calculation, and obstacle avoidance.

[SLAM Mapping](./slam) explains how to run SLAM algorithms for mapping. This section introduces how to use Nav2 for navigation based on the created map. Similarly, use Gazebo on the PC to create a virtual environment and a car, use Rviz2 to set the navigation destination, and run the Nav2 program for navigation using the Horizon RDK.

## Supported Platforms

| Platform                | Execution      | Example Functionality             |
| ----------------------- | -------------- | --------------------------------- |
| RDK X3, RDK X3 Module    | Ubuntu 20.04   | Start the simulation environment on the PC and start the navigation function on the Horizon RDK, and finally display the navigation effect through Rviz2 |

## Prerequisites

### Horizon RDK Platform

1. The Horizon RDK has been burned with the Ubuntu 20.04 image provided by Horizon.

2. The Horizon RDK has successfully installed tros.b.

3. After tros.b is successfully installed, install Nav2.

   ```shell
   sudo apt update 
   sudo apt install ros-foxy-navigation2
   sudo apt install ros-foxy-nav2-bringup
   ```

4. The PC on the same network segment as the Horizon RDK has installed Ubuntu 20.04 system, ROS2 Foxy desktop version, simulation environment Gazebo, and data visualization tool Rviz2.

   Reference for ROS2 Foxy installation: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

   After successful installation of ROS2 Foxy on the PC, install the Gazebo and Turtlebot3 related packages as well as navigation2, using the following commands:

   ```shell
   sudo apt install ros-foxy-gazebo-*
   sudo apt install ros-foxy-turtlebot3*
   
   sudo apt install ros-foxy-navigation2
   sudo apt install ros-foxy-nav2-bringup
   ```## User Guide

### Horizon RDK Platform

This section describes how to set up a simulation environment on the PC side, how to set the navigation destination, and how to run the navigation function with Horizon RDK and view the navigation result.

1. Start the gazebo simulation environment on the **PC side**.

   ```shell
   source /opt/ros/foxy/setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

   The simulation environment is shown in the following image:

   ![](./image/nav2/gazebo.png)

2. Start the navigation function on the **Horizon RDK**.

   ```shell
   source /opt/tros/setup.bash

   ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/opt/ros/foxy/share/nav2_bringup/maps/turtlebot3_world.yaml
   ```

3. Start the Rviz2 tool on the **PC side**.

   ```shell
   source /opt/ros/foxy/setup.bash
   ros2 launch nav2_bringup rviz_launch.py
   ```

   The Rviz2 interface is shown in the following image:

   ![](./image/nav2/rviz.png)

4. Set the initial position and orientation of the robot in Rviz2.

   After Rviz2 is launched, the robot does not know where it is initially. By default, Nav2 waits for the user to provide an approximate initial position for the robot. Refer to the robot's position in Gazebo and find that position on the map. Set the initial position of the robot by clicking the "2D Pose Estimate" button in Rviz2 and then clicking on the estimated position of the robot on the map. The initial movement direction of the robot can be set by dragging the clicked position forward. The process is shown in the following image:

   ![](./image/nav2/rviz_init.png)

   Once the initial position of the robot is set, the coordinate transformation tree will be established and Nav2 will be fully activated and ready. The robot and point cloud can be seen at this time.

   ![](./image/nav2/rviz_start.png)

5. Set the destination in Rviz2.

   Click the "Navigation2 Goal" button and select a destination.Here is the translation of the Chinese parts in the content, while preserving the original format and content:

   ![](./image/nav2/rviz_goal.png)

   Now you can see the robot is moving.

## Result Analysis

The navigation effect is shown in the figure below:

![](./image/nav2/rviz_nav2.gif)