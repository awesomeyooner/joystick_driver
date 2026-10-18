[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![ubuntu24.04](https://img.shields.io/badge/-UBUNTU_24.04-orange?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/noble/)
[![jazzy](https://img.shields.io/badge/-JAZZY-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/index.html)

# joystick_driver
ROS 2 package that acts as an abstraction layer for the standard `joy` node

## Prerequisites

```bash
# Install necessary components
$ sudo apt install -y joystick \
  jstest-gtk \
  evtest \
  libyaml-cpp-dev

# Run This to test every device
$ sudo evtest

# Run this and look for specific key words to find which event corresponds to your device
$ cat /proc/bus/input/devices
```

## Usage

### Setup

```bash
# In /ros2_ws/src
$ git clone https://github.com/awesomeyooner/joystick_driver.git && cd joystick_driver
$ git submodule update --init --recursive
```

Please create a configuration `.yaml` file in the `\config` folder. Here is an example:

`joystick.yaml`

```yaml
joy_node:
  ros__parameters:
    deadzone: 0.05

joystick_teleop:
  ros__parameters:
    use_sim_time: false
    joystick_type: "ps4" # implemented ones are: "ps4" and "switch" so far

    twist_topic: "/cmd_vel"

    # Only used when `twist_type = "both"`. When `twist_type = "stamped"`, the topic used will be `twist_topic`
    # This param only exists to differentiate the topics when both types are published
    stamped_topic: "/cmd_vel/stamped"

    twist_type: "both"
    # - "stamped" Publishes only TwistStamped on `twist_topic`
    # - "unstamped" Publishes only Twist on `twist_topic`
    # - "both" Publishes both TwistStamped and Twist on `twist_topic/stamped` and `twist_topic` respectively

    max_tangential_velocity: 1.0 # 1.0
    max_angular_velocity: 1.0 # 3.14
```

To create custom mappings, run the `joy_node` in one terminal then run the `create_mappings` node

```bash
# Run joy_node
ros2 run joy joy_node

# Run create_mappings
ros2 run joystick_driver create_mappings
```

Then follow the steps on the terminal. This will create `joystick_mappings.yaml` in the current directory that you called the `ros2 run` command in. You could either copy paste the contents into your main `.yaml` file or just use that file as a whole.

To launch, create a `.launch.py` file in the `\launch` folder

`joystick.launch.py`

```python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    # CHANGE ME
    package = "robot_hardware"

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Whether or not to use sim time. Defaults to false",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # CHANGE ME
    params_file = os.path.join(get_package_share_directory(package), 'config','joystick.yaml')

    launch_file = os.path.join(
        get_package_share_directory("joystick_driver"),
        "launch",
        "joystick.launch.py"
    )

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": use_sim_time
        }.items()
    )

    nodes = [
        joystick_launch
    ]

    return LaunchDescription(declared_arguments + nodes)

```

You can launch using the following

```bash
# Standard Launch
$ ros2 launch <my_package> joystick.launch.py
$ ros2 launch joystick_driver joystick.launch.py

# Launch with use_sim_time = true
$ ros2 launch <my_package> joystick.launch.py use_sim_time:=true
$ ros2 launch joystick_driver joystick.launch.py use_sim_time:=true
```

## Usage with Docker

When using docker, you can put a volume mount on all your input devices

```bash
docker run \
  -v /dev/input:/dev/input \
  --device-cgroup-rule='c 13:* rmw' \
  image:tag
```

- `--volume /dev/input:/dev/input` Will put a volume mount on input devices
- `--device-cgroup-rule='c 13:* rmw'` Will allow permission to use devices