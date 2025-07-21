> ROS2 Fork repo maintainer: [Kmedrano101](https://github.com/Kmedrano101)

## About the `jetson-dev` Branch

This branch is tailored for running the Livox ROS2 driver on ARM CPU architectures, such as NVIDIA Jetson devices. It includes optimizations and configurations specific to ARM-based development environments.

### Getting Started

#### 1. Clone the Repository

Open a terminal and navigate to your ROS2 workspace's `src` directory (e.g., `cd ~/ros2_ws/src`). Then run:

```bash
git clone -b jetson-dev https://github.com/Kmedrano101/livox_ros_driver2.git
```

#### 2. Install Dependencies

From the root of your ROS2 workspace (e.g., `cd ~/ros2_ws`), update and install the required dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### 3. Build the Package

Still in your ROS2 workspace root, build the package and source the setup file:

```bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DHUMBLE_ROS=humble
source install/setup.bash
```
> **Note:** This package requires the Livox SDK2. Please refer to the [Livox SDK2 README](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md) for installation and setup instructions before proceeding.

#### 4. Run the Package

After building, you can launch the driver (from anywhere with the workspace sourced):

```bash
ros2 launch livox_ros_driver2 multiple_lidars.launch.py
```

This will launch the driver with support for multiple Livox lidars on your ARM-based device.