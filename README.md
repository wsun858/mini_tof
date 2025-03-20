# Mini ToF

ROS 2 package for interfacing with miniature ToF sensors without any hardware or firmware expertise required. Unlike existing packages, this package includes support for capturing raw histogram data from the sensors, which has been studied in academic work for many applications like object detection [[1](https://cpsiff.github.io/papers/using_a_distance_sensor/index.html)], 3D reconstruction[[2](https://cpsiff.github.io/papers/towards_3d_vision/index.html), [3](https://cpsiff.github.io/papers/unlocking_proximity_sensors/index.html), [4](https://nikhilbehari.github.io/bls3d-web/)], and even seeing around corners[[5](https://zheng-shi.github.io/papers/CheapSPAD_main.pdf), [6](https://camera-culture.github.io/nlos-aided-autonomous-navigation/)].

Currently supported sensors:
* AMS TMF882X (TMF8820, TMF8825, TMF8828)
* ST VL53L8CH

See the [mini_tof_firmware](https://github.com/uwgraphics/mini_tof_firmware) repository for information about supported hardware (sensors and microcontollers).

**Related Packages**
[mini_tof_interfaces](https://github.com/uwgraphics/mini_tof_interfaces) is a ROS 2 package containing the ROS 2 interface definitions for this package.
[mini_tof_firmware](https://github.com/uwgraphics/mini_tof_firmware) is a GitHub repository containing microcontroller firmware necessary for using this package.

## Installation

### Install ROS Package

#### Option 1: Install pre-built binary from command line
*This option is coming soon. For now, you must build from source.*

#### Option 2: Build from Source
1. Clone this repo and the interfaces package to the `src` directory of your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone git@github.com:uwgraphics/mini_tof.git
    git clone git@github.com:uwgraphics/mini_tof_interfaces.git
    ```
2. Install python dependencies using rosdep:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

   > **Note:** If you encounter issues with `rosdep`, you can also install packages manually. Refer to the `package.xml` files in the `mini_tof` and `mini_tof_interfaces` packages for a list of dependencies.

3. Build the workspace
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

### Hardware, Microcontroller Setup, and Firmware
To use this package, a microcontroller must be connected to the ToF sensor and run firmware that communicates with the sensor and ROS 2. Follow the steps in the README for the [mini_tof_firmware](https://github.com/uwgraphics/mini_tof_firmware) repository to set up the microcontroller and upload the firmware.

## Usage
The `mini_tof` package is meant to be a simple, lightweight package which passes along information reported by miniature time-of-flight sensors to ROS 2 topics. There are only two nodes in the package:

1. `tof_publisher`: Publishes data reported by the sensor (e.g. distance, ambient light, histograms) to ROS 2 topics.
2. `tof_visualizer`: Visualizes the data reported by the sensor in real time using PyQtGraph.

### Command line
Launch a publisher node for the TMF882X sensor with:
```bash
ros2 run mini_tof tof_publisher --ros-args -p mcu_port:=/dev/ttyACM0 -p sensor_model:=TMF882X
```
Or launch a publisher node for the VL53L8CH sensor with:
```bash
ros2 run mini_tof tof_publisher --ros-args -p mcu_port:=/dev/ttyUSB0 -p sensor_model:=VL53L8CH -p num_zones:=16
```
Check that messages are getting published to the `mini_tof_data` topic with:
```bash
ros2 topic echo /mini_tof_data
```
Visualize the reported data in real time with:
```bash
ros2 run mini_tof tof_visualizer
```

### Launch files
Like any ROS node, the publisher and visualizer can also be launched via launch files. An example launch file for the VL53L8CH sensor looks like:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_tof',
            name='tof_publisher',
            executable='tof_publisher',
            output='screen',
            parameters=[{
                'mcu_port': '/dev/ttyUSB0',
                'sensor_model': 'VL53L8CH',
                'num_zones': 16
            }]
        ),
        Node(
            package='mini_tof',
            name='tof_visualizer',
            executable='tof_visualizer',
            output='screen'
        ),
    ])
```

# Troubleshooting
If you run into any problems, please [open an issue](https://github.com/uwgraphics/mini_tof/issues/new) on this repository and an author will help you out.