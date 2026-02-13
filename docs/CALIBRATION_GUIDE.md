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

## Current Configuration (VERIFIED WORKING)

### LiDAR 1 (L1) - Left Sensor ✓ CONFIRMED
- **Position**: [0.0, 0.11, 0.0] meters
  - 11cm to the left (Y+)
- **Orientation**:
  - Roll: -90° (Z-axis points LEFT/outward, per MID-360 spec)
  - Pitch: 0°
  - Yaw: 0° (facing forward)
- **IP**: 192.168.1.10
- **Driver Topic**:
  - Single mode: `/livox/lidar` (when multi_topic=0)
  - Multi mode: `/livox/lidar_192_168_1_10` (when multi_topic=1)
- **Calibration Topic**: `/lidar_L1/pointcloud` (republished with correct frame)
- **Driver Extrinsics** (in config JSON):
  - roll: -90.0, pitch: 0.0, yaw: 0.0
  - x: 0, y: 110 (mm), z: 0
- **Python Inverse Transform**: roll: +90.0° (to undo driver transform)
- **Status**: ✅ Verified in RViz - pose and orientation match physical sensor
- **Calibration Test Results**:
  - BEFORE inverse transform: x=-1.028, y=0.122, z=-0.498 (driver-transformed coordinates)
  - AFTER inverse transform: x=-0.107, y=0.494, z=0.416 (sensor frame coordinates)
  - Transform successfully reverts driver extrinsics, ready for TF visualization

### LiDAR 2 (L2) - Right Sensor
- **Position**: [0.0, -0.11, 0.0] meters
  - 11cm to the right (Y-)
- **Orientation**:
  - Roll: -90° (Z-axis points RIGHT/outward, per MID-360 spec)
  - Pitch: 0°
  - Yaw: 180° (facing backward)
- **IP**: 192.168.1.18
- **Driver Topic**:
  - Multi mode: `/livox/lidar_192_168_1_18` (when multi_topic=1)
- **Calibration Topic**: `/lidar_L2/pointcloud` (republished with correct frame)
- **Driver Extrinsics** (in config JSON):
  - roll: -90.0, pitch: 0.0, yaw: 180.0
  - x: 0, y: -110 (mm), z: 0
- **Python Inverse Transform**: roll: +90.0°, yaw: -180.0° (to undo driver transform)
- **Status**: 🔄 To be verified (use same approach as L1)

## How the Calibration System Works

The calibration system uses a two-step transformation approach:

1. **Driver applies extrinsics**: The Livox driver (in `multiple_netconfigs.json` or `mid360_netconfigs.json`) applies the extrinsic transformation to incoming point clouds. This ensures FAST-LIO and other packages receive correctly oriented data.

2. **Python node applies inverse**: The `lidar_tf_calibration.py` node subscribes to the driver's output and applies the **inverse transformation** to get back to the sensor's original frame. It then republishes with the correct `frame_id` so RViz/TF can apply the visualization transform.

**Why this approach?**
- ✅ Driver extrinsics remain in config (needed by FAST-LIO)
- ✅ Point clouds align with TF frames in RViz
- ✅ Easy to verify calibration visually before running SLAM

**Example for L1:**
- Driver config: roll=-90°, y=110mm → transforms sensor data
- Python inverse: roll=+90° → reverts to sensor frame
- TF transform: roll=-90°, y=0.11m → RViz applies visualization transform
- Result: Point cloud aligns perfectly with TF frame

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

## Configuration Verification Summary

### ✅ Verified Configuration Files

**multiple_netconfigs.json** (for dual LiDAR with multi_topic=1):
```json
{
  "MID360": {
    "host_net_info": [
      {
        "lidar_ip": ["192.168.1.10"],
        "host_ip": "192.168.1.124",
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501
      },
      {
        "lidar_ip": ["192.168.1.18"],
        "host_ip": "192.168.1.124",
        "cmd_data_port": 56102,
        "push_msg_port": 56202,
        "point_data_port": 56302,
        "imu_data_port": 56402,
        "log_data_port": 56502
      }
    ]
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.10",
      "extrinsic_parameter": {
        "roll": -90.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 110,
        "z": 0
      }
    },
    {
      "ip": "192.168.1.18",
      "extrinsic_parameter": {
        "roll": -90.0,
        "pitch": 0.0,
        "yaw": 180.0,
        "x": 0,
        "y": -110,
        "z": 0
      }
    }
  ]
}
```

**mid360_netconfigs.json** (for single L1 debugging):
```json
{
  "MID360": {
    "host_net_info": [
      {
        "lidar_ip": ["192.168.1.10"],
        "host_ip": "192.168.1.124",
        "cmd_data_port": 56101,
        "point_data_port": 56301,
        "imu_data_port": 56401
      }
    ]
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.10",
      "extrinsic_parameter": {
        "roll": -90.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 110,
        "z": 0
      }
    }
  ]
}
```

## Verification Checklist

- [x] L1 configuration verified in RViz
- [x] L1 point cloud aligns with TF frame
- [x] L1 pose and orientation match physical sensor
- [ ] L2 configuration to be verified (same approach as L1)
- [ ] Both LiDAR frames visible in RViz (dual mode)
- [ ] L1 is positioned to the left of base_link (Y+)
- [ ] L2 is positioned to the right of base_link (Y-)
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
