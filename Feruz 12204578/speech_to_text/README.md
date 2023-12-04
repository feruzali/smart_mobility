# STT (Speech To Text) for ROS2

## Feruz 12204578

STT for ROS 2 using JSGF grammars.

## Installation

This module requires `simple_node` open-source module, install it first.

```shell
git clone https://github.com/uleroboticsgroup/simple_node
```

```shell
cd ~/ros2_ws/src

# dependencies
sudo apt-get install -y python-dev-is-python3 libportaudio2 libportaudiocpp0 portaudio19-dev libasound-dev swig
cd speech_to_text
pip3 install -r requirements.txt
python3 ./nltk_download.py

# colcon
cd ~/ros2_ws
colcon build
```

## Usage

### Launch

```shell
ros2 launch speech_to_text speech_to_text.launch.py
```

### Shell Example

```shell
ros2 action send_goal /speech_to_text/listen_once speech_to_text_msgs/action/ListenOnce {}
```
