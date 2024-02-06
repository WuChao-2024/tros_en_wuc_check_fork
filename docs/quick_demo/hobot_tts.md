---
sidebar_position: 8
---

# 2.8 Text to Speech

## Function Introduction

This section describes how to convert a text into a speech signal and play it through an audio output interface.

Code repository: <https://github.com/HorizonRDK/hobot_tts.git>

## Supported Platforms

| Platform | Operation System | Example Function               |
| -------- | ---------------- | ------------------------------ |
| RDK X3   | Ubuntu 20.04    | Subscribe to text messages, convert them into speech data, and play them out |

**Note: Only supports RDK X3, RDK X3 Module is not supported yet.**

## Preparation

### Horizon RDK platform

1. Horizon RDK has been flashed with Ubuntu 20.04 system image provided by Horizon.
2. TogetheROS.Bot has been successfully installed on Horizon RDK.
3. An audio driver board compatible with Horizon has been obtained, and the environment has been set up according to [Smart Voice section](../boxs/box_adv#Smart Voice).
4. Connect the audio board's headphone interface with headphones or speakers.

## Usage

### Horizon RDK platform

1. For the first run, you need to download the models file and extract it. The detailed commands are as follows:

    ```bash
    wget http://sunrise.horizon.cc/tts-model/tts_model.tar.gz
    sudo tar -xf tts_model.tar.gz -C /opt/tros/lib/hobot_tts/
    ```

2. Run the following command to check if the audio device is normal:

    ```bash
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
    ```

    If a similar audio playback device like `pcmC0D1p` appears, it means the device is working fine.

3. Start the hobot_tts program.

```bash
source /opt/tros/setup.bash

# Disable debug print information
export GLOG_minloglevel=1

ros2 run hobot_tts hobot_tts
```

Note: If the audio playback device is not `pcmC0D1p`, you need to use the `playback_device` parameter to specify the playback audio device. For example, if the audio playback device is `pcmC1D1p`, the launch command should be: `ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="hw:1,1"`

4. Open a new terminal and use the echo command to publish a topic

   ```bash
   source /opt/tros/setup.bash
   ros2 topic pub --once /tts_text std_msgs/msg/String "{data: ""Do you know the horizon? Yes, I know the horizon. It is a line that extends from the ground to the sky, defining the boundary between the ground and the sky.""}"
   ```

5. You can hear the playback sound from your headphones or speakers.

## Notes

Currently, only Chinese and English text content are supported. Do not publish text messages in other languages.