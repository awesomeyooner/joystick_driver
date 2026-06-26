[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![ubuntu22](https://img.shields.io/badge/-UBUNTU_22.04-orange?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/jammy/)
[![humble](https://img.shields.io/badge/-HUMBLE-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/index.html)

# joystick_driver
ROS 2 package that acts as an abstraction layer for the standard `joy` node

## Prerequisites

```bash
# Install necessary components
$ sudo apt install -y joystick \
  jstest-gtk \
  evtest

# Run This to test every device
$ sudo evtest

# Run this and look for specific key words to find which event corresponds to your device
$ cat /proc/bus/input/devices
```

## Usage

Please create a configuration `.yaml` file in the `\config` folder. Here is an example:

`joystick.yaml`

```yaml
joy_node:
  ros__parameters:
    deadzone: 0.05

joystick_teleop:
  ros__parameters:
    use_sim_time: true
    joystick_type: "ps4" 
    # implemented ones are: 
    # "ps4"
    # "gamesir"
    # "xbox"

    max_tangential_velocity: 1.0 # m/s
    max_angular_velocity: 3.14 # rad/s
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
    package = "my_package"

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
            "params_file": params_file
        }.items()
    )

    nodes = [
        joystick_launch
    ]

    return LaunchDescription(nodes)

```

and launch with `ros2 launch <my_package> joystick.launch.py`

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