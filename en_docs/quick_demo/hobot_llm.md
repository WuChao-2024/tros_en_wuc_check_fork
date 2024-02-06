---
sidebar_position: 9
---

# 2.9 Large Language Model

## Function Introduction

This section introduces how to experience edge-side Large Language Model (LLM) on the Horizon RDK platform.

Code repository: <https://github.com/HorizonRDK/hobot_llm.git>

## Supported Platforms

| Platform                       | Running Method | Example Function |
| ------------------------------ | -------------- | ---------------- |
| RDK X3, RDK X3 Module (4GB RAM) | Ubuntu 20.04   | Edge-side LLM Experience |

**Note: Only supports RDK X3, RDK X3 Module with 4GB RAM version.**

## Preparation

### Horizon RDK Platform

1. Horizon RDK with 4GB RAM version.
2. Horizon RDK has been flashed with the provided Ubuntu 20.04 system image.
3. Horizon RDK has successfully installed TogetheROS.Bot.
4. Install transformers, the command is `pip3 install transformers -i https://pypi.tuna.tsinghua.edu.cn/simple`.
5. Update hobot-dnn, the command is `sudo apt update; sudo apt install hobot-dnn`.

## Usage

### Horizon RDK Platform

Before running the program, you need to download the model file and extract it, the commands are as follows:

```bash
# Download the model file
wget http://sunrise.horizon.cc/llm-model/llm_model.tar.gz

# Extract
sudo tar -xf llm_model.tar.gz -C /opt/tros/lib/hobot_llm/
```

Use the command `srpi-config` to modify the ION memory size to 1.7GB. The configuration method can be referred to the "Performance Options" section of the RDK User Manual Configuration Tool `srpi-config` Guide [Performance Options](https://developer.horizon.cc/documents_rdk/configuration/srpi-config#performance-options).

After restarting, set the CPU maximum frequency to 1.5GHz and the scheduling mode to `performance`, the commands are as follows:

```bash
sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
```sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'

Currently, there are two ways to experience it. One is to directly input text in the terminal for chat interaction, and the other is to subscribe to text messages and then publish the results in text format.

#### Terminal Interaction Experience

```bash
source /opt/tros/setup.bash

ros2 run hobot_llm hobot_llm_chat
```

After the program starts, you can chat directly with the robot in the current terminal.

#### Subscribe and Publish Experience

1. Start hobot_llm

    ```bash
    source /opt/tros/setup.bash

    ros2 run hobot_llm hobot_llm
    ```

2. Open a new terminal to subscribe to the output result topic

    ```bash
    source /opt/tros/setup.bash

    ros2 topic echo /text_result
    ```

3. Open a new terminal to publish a message

    ```bash
    source /opt/tros/setup.bash

    ros2 topic pub --once /text_query std_msgs/msg/String "{data: ""What is the capital of China?""}"
    ```

After sending the message, you can check the output result in the subscribed terminal.

## Note

Make sure the development board has 4GB of memory and modify the ION memory size to 1.7GB, otherwise the model loading will fail.