# How to Restart Livox ROS Driver 2 with New Configuration

## Issue
The livox_ros_driver2 is currently running with old parameters and publishing only a single topic `/livox/lidar` instead of separate topics for each sensor.

## Solution
After rebuilding the package, you need to restart the driver with the new configuration.

## Steps

### 1. Stop the current driver (if running)

Find the running livox driver process:
```bash
ps aux | grep livox_ros_driver2_node
```

Kill the process:
```bash
pkill -9 livox_ros_driver2_node
```

Or if using a launch file, press `Ctrl+C` in the terminal where it's running.

### 2. Source the updated workspace

```bash
cd /home/jetson/ros2_ws
source install/setup.bash
```

### 3. Launch the driver with the new configuration

```bash
ros2 launch livox_ros_driver2 multiple_lidars.launch.py
```

### 4. Verify the topics are being published correctly

In a new terminal:
```bash
source /home/jetson/ros2_ws/install/setup.bash
ros2 topic list | grep livox
```

**Expected output:**
```
/livox/imu_192_168_1_10
/livox/imu_192_168_1_18
/livox/lidar_192_168_1_10
/livox/lidar_192_168_1_18
```

### 5. Check topic details

Verify L1 topic:
```bash
ros2 topic info /livox/lidar_192_168_1_10
```

Verify L2 topic:
```bash
ros2 topic info /livox/lidar_192_168_1_18
```

### 6. Monitor data rate

```bash
ros2 topic hz /livox/lidar_192_168_1_10
ros2 topic hz /livox/lidar_192_168_1_18
```

Both should show approximately 10 Hz (configured in livox_params.yaml).

## Troubleshooting

### Still seeing only `/livox/lidar` topic?

Check that the parameter is being read correctly:
```bash
ros2 param list /livox_lidar_publisher
ros2 param get /livox_lidar_publisher multi_topic
```

Expected: `Integer value is: 1`

If it shows `0`, the parameters file isn't being loaded. Check:
```bash
ros2 launch livox_ros_driver2 multiple_lidars.launch.py --show-args
```

### No topics at all?

Check if the sensors are connected:
```bash
ping 192.168.1.10
ping 192.168.1.18
```

### Only one sensor publishing?

Check the multiple_netconfigs.json file:
```bash
cat /home/jetson/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/multiple_netconfigs.json
```

Verify both sensors (IPs 192.168.1.10 and 192.168.1.18) are listed.

## Launch with FAST-LIO

Once the topics are publishing correctly, you can launch FAST-LIO:

```bash
ros2 launch fast_lio_ros2 mapping.launch.py config:=dual_mid360_mine
```

## Quick Verification Script

Create this script to quickly check the configuration:

```bash
#!/bin/bash
echo "=== Livox Topics Check ==="
ros2 topic list | grep livox
echo ""
echo "=== Multi-topic Parameter ==="
ros2 param get /livox_lidar_publisher multi_topic 2>/dev/null || echo "Node not running"
echo ""
echo "=== Topic Rates ==="
timeout 5 ros2 topic hz /livox/lidar_192_168_1_10 2>/dev/null || echo "L1 topic not publishing"
timeout 5 ros2 topic hz /livox/lidar_192_168_1_18 2>/dev/null || echo "L2 topic not publishing"
```

## Configuration Files Updated

The following files were updated and rebuilt:

1. **livox_params.yaml**
   - `multi_topic: 1` (enables separate topics)
   - `user_config_path: "<package_share>/config/multiple_netconfigs.json"`

2. **multiple_netconfigs.json**
   - L1 extrinsics: y=110mm (left position)
   - L2 extrinsics: y=-110mm (right position)

3. **fast_lio_ros2 config**
   - Topics: `/livox/lidar_192_168_1_10` and `/livox/lidar_192_168_1_18`
   - Extrinsics match the driver configuration

All configurations are synchronized and ready to use!
