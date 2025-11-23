# Dual LiDAR Calibration Guide

This guide explains how to use the LiDAR calibration visualization tool to verify and adjust the extrinsic parameters for the dual Livox MID-360 setup.

## Overview

The calibration system consists of:
- **lidar_tf_calibration.py**: Python node that publishes TF transforms
- **lidar_calibration_viz.launch.py**: Launch file for visualization
- **lidar_calibration.rviz**: RViz configuration file

## Quick Start

### 1. Build the packages

```bash
cd /home/jetson/ros2_ws
colcon build --packages-select livox_ros_driver2 fast_lio_ros2
source install/setup.bash
```

### 2. Launch calibration visualization

```bash
ros2 launch livox_ros_driver2 lidar_calibration_viz.launch.py
```

This will:
- Start the TF calibration node
- Open RViz with the calibration configuration

### 3. What you should see in RViz

- **Grid**: Reference grid on XY plane
- **TF frames**: Three coordinate frames:
  - `base_link` (blue/red/green axes - largest)
  - `lidar_L1` (left sensor - medium axes)
  - `lidar_L2` (right sensor - medium axes)
- **Axes displays**: Additional axis markers for each frame

## Current Configuration

### LiDAR 1 (L1) - Left Sensor
- **Position**: [0.0, 0.11, 0.0] meters
  - 11cm to the left (Y+)
- **Orientation**:
  - Roll: 90° (standard Livox mounting)
  - Pitch: 0°
  - Yaw: 0° (facing forward)
- **IP**: 192.168.1.10
- **Topic**: `/livox/lidar_192_168_1_10`

### LiDAR 2 (L2) - Right Sensor
- **Position**: [0.0, -0.11, 0.0] meters
  - 11cm to the right (Y-)
- **Orientation**:
  - Roll: 90° (standard Livox mounting)
  - Pitch: 0°
  - Yaw: 180° (facing backward)
- **IP**: 192.168.1.18
- **Topic**: `/livox/lidar_192_168_1_18`

## Adjusting Calibration Parameters

### Method 1: Launch Arguments (Quick testing)

You can override parameters when launching:

```bash
ros2 launch livox_ros_driver2 lidar_calibration_viz.launch.py \
  l1_y:=0.12 \
  l2_y:=-0.12 \
  l1_yaw:=5.0
```

Available parameters:
- `l1_x`, `l1_y`, `l1_z`: L1 position (meters)
- `l1_roll`, `l1_pitch`, `l1_yaw`: L1 orientation (degrees)
- `l2_x`, `l2_y`, `l2_z`: L2 position (meters)
- `l2_roll`, `l2_pitch`, `l2_yaw`: L2 orientation (degrees)

### Method 2: Edit Configuration Files (Permanent changes)

After verifying the correct values in RViz, update the configuration files:

#### 1. Update livox_ros_driver2 config
Edit: `/home/jetson/ros2_ws/src/livox_ros_driver2/config/multiple_netconfigs.json`

**Important**: Values must be in **millimeters** and as **integers**:
```json
"extrinsic_parameter": {
  "roll": 90.0,
  "pitch": 0.0,
  "yaw": 0.0,
  "x": 0,
  "y": 110,      // 110mm = 0.11m
  "z": 0
}
```

#### 2. Update FAST-LIO config
Edit: `/home/jetson/ros2_ws/src/fast_lio_ros2/config/dual_mid360_mine.yaml`

Values in **meters** as **floats**:
```yaml
extrinsic_T: [0.0, 0.11, 0.0]  # [x, y, z] meters
extrinsic_R: [ 1.00000,  0.00000,  0.00000,
               0.00000,  0.00000, -1.00000,
               0.00000,  1.00000,  0.00000]
```

## Verification Checklist

- [ ] Both LiDAR frames are visible in RViz
- [ ] L1 is positioned to the left of base_link
- [ ] L2 is positioned to the right of base_link
- [ ] L1 X-axis (red) points forward
- [ ] L2 X-axis (red) points backward (180° from L1)
- [ ] Both sensors have correct roll (Z-axis pointing sideways)
- [ ] Distance between L1 and L2 is 22cm (0.22m)

## Understanding the Coordinate Frames

### base_link (Drone body frame)
- X-axis (red): Forward
- Y-axis (green): Left
- Z-axis (blue): Up

### lidar_L1 / lidar_L2 (Livox sensor frames)
After 90° roll rotation:
- X-axis (red): Scan direction (forward for L1, backward for L2)
- Y-axis (green): Perpendicular to scan
- Z-axis (blue): Perpendicular to mounting surface

## Visualizing with Live LiDAR Data

To see the actual point clouds along with the TF frames:

1. Start the livox driver:
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

2. In RViz, enable the point cloud displays:
   - L1_PointCloud (topic: `/livox/lidar_192_168_1_10`)
   - L2_PointCloud (topic: `/livox/lidar_192_168_1_18`)

3. Verify that point clouds align with their respective coordinate frames

## Troubleshooting

### TF frames not showing
- Check that the calibration node is running: `ros2 node list`
- Verify TF is being published: `ros2 run tf2_ros tf2_echo base_link lidar_L1`

### Point clouds not visible
- Ensure livox driver is running
- Check topic names match: `ros2 topic list | grep livox`
- Verify `multi_topic: 1` in livox_params.yaml

### Misalignment between point clouds
- Re-measure physical sensor positions
- Verify rotation values match actual mounting
- Use offline calibration tools for fine-tuning

## Advanced: Offline Calibration

For precise calibration, you can use tools like:
- **GICP-based calibration**: Align point clouds in post-processing
- **LI-Calib**: LiDAR-IMU calibration toolkit
- **Manual measurement**: CAD models or precise measuring tools

## Related Files

- Node: `livox_ros_driver2/scripts/lidar_tf_calibration.py`
- Launch: `livox_ros_driver2/launch/lidar_calibration_viz.launch.py`
- RViz config: `livox_ros_driver2/rviz_cfg/lidar_calibration.rviz`
- Driver config: `livox_ros_driver2/config/multiple_netconfigs.json`
- FAST-LIO config: `fast_lio_ros2/config/dual_mid360_mine.yaml`

## Support

For issues or questions:
- Check ROS 2 logs: `ros2 run rqt_console rqt_console`
- View TF tree: `ros2 run tf2_tools view_frames`
- Monitor transforms: `ros2 run rqt_tf_tree rqt_tf_tree`
