# 🚀 Livox ROS2 Driver

> 👤 **ROS2 Fork repo maintainer:** [Kmedrano101](https://github.com/Kmedrano101)

---

## 📌 About the `jetson-dev` Branch

This branch is tailored for running the Livox ROS2 driver on **ARM CPU architectures**, such as NVIDIA Jetson devices. It includes optimizations and configurations specific to ARM-based development environments.

---

## 🛠️ Getting Started

### 📥 1. Clone the Repository

Open a terminal and navigate to your ROS2 workspace's `src` directory (e.g., `cd ~/ros2_ws/src`). Then run:

```bash
git clone -b jetson-dev https://github.com/Kmedrano101/livox_ros_driver2.git
```

### 📦 2. Install Dependencies

From the root of your ROS2 workspace (e.g., `cd ~/ros2_ws`), update and install the required dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 🔨 3. Build the Package

Still in your ROS2 workspace root, build the package and source the setup file:

```bash
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

> ⚠️ **Important:** This package requires the Livox SDK2. Please refer to the [Livox SDK2 README](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md) for installation and setup instructions before proceeding.

### ▶️ 4. Run the Package

After building, you can launch the driver (from anywhere with the workspace sourced):

```bash
ros2 launch livox_ros_driver2 multiple_lidars.launch.py
```

This will launch the driver with support for multiple Livox lidars on your ARM-based device.

---

## ⚙️ Configuration for Dual Livox MID-360 Setup

This package is configured to work with **two Livox MID-360 sensors** 🎯 using the following configuration files:

---

### 📁 Key Configuration Files

#### 📄 1. `livox_params.yaml`
**Location:** `config/livox_params.yaml`

**Key Parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `xfer_format` | `0` | 📊 Uses standard PointCloud2 format (PointXYZRTL) |
| `multi_topic` | `0` | 📡 All LiDARs share the same topic |
| `data_src` | `0` | 🔌 Data source from LiDAR |
| `publish_freq` | `10` | ⏱️ Publishing frequency in Hz (adjustable: 5, 10, 20, 50 Hz) |
| `frame_id` | `"livox_frame"` | 🎯 TF frame name for the point cloud |
| `user_config_path` | `<package_share>/config/...` | 🗂️ Points to network configuration file |

---

#### 📄 2. `multiple_test.json` ✅ (Current Working Configuration)
**Location:** `config/multiple_test.json`

This is the **latest verified working configuration** for dual MID-360 setup.

---

### 🌐 Network Configuration

#### 🔗 IP Address Setup

| Device | IP Address | Description |
|--------|------------|-------------|
| 🖥️ **Host Computer** | `192.168.1.124` | Your computer running ROS2 |
| 🔴 **L1 (Lidar 1)** | `192.168.1.10` | First MID-360 sensor |
| 🔵 **L2 (Lidar 2)** | `192.168.1.18` | Second MID-360 sensor |

> 💡 **Pro Tip - QR Code Identification:**
>
> The host part of the IP addresses (`.10` and `.18`) correspond to the **last 2 digits of the QR code** printed on the physical Livox sensor. This helps identify which physical sensor is which:
> - 📱 Sensor with QR ending in `10` → `192.168.1.10`
> - 📱 Sensor with QR ending in `18` → `192.168.1.18`

---

### 🔌 Port Configuration per Lidar

| Port Type | 🔴 L1 (192.168.1.10) | 🔵 L2 (192.168.1.18) |
|-----------|---------------------|---------------------|
| 💻 Command Data Port | 56101 | 56102 |
| 📨 Push Message Port | 56201 | 56202 |
| 📍 Point Data Port | 56301 | 56302 |
| 🧭 IMU Data Port | 56401 | 56402 |
| 📝 Log Data Port | 56501 | 56502 |

---

### 🧭 Extrinsic Parameters

Each lidar has extrinsic calibration parameters defining its orientation:

| Lidar | Roll | Pitch | Yaw | Position (x, y, z) | Notes |
|-------|------|-------|-----|-------------------|-------|
| 🔴 **L1** (192.168.1.10) | 90° | 0° | 0° | (0, 0, 0) | ➡️ Forward facing |
| 🔵 **L2** (192.168.1.18) | 90° | 0° | 180° | (0, 0, 0) | ⬅️ Backward facing |

Both sensors are positioned at the origin with different orientations to provide 360° coverage.

---

### 🔧 LiDAR Type Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| `lidar_type` | `8` | 🎯 Corresponds to MID-360 sensor type |
| `pcl_data_type` | `1` | 📊 Point cloud data format |
| `pattern_mode` | `0` | 🔄 Scan pattern mode |

---

## 🌐 Network Setup Requirements

To use this configuration, follow these steps:

1. 🖥️ **Set your host computer's IP** to `192.168.1.124` on the network interface connected to the LiDARs
   ```bash
   # Example for setting static IP on Ubuntu
   sudo nmcli con mod <connection-name> ipv4.addresses 192.168.1.124/24
   sudo nmcli con mod <connection-name> ipv4.method manual
   sudo nmcli con up <connection-name>
   ```

2. 🔌 **Connect both LiDARs** to the same network switch/router

3. 📱 **Verify LiDAR IPs** match the QR code endings:
   - Check the physical QR code on each sensor
   - The last 2 digits should match the host part of the IP
   - Example: QR ending in `10` → IP `192.168.1.10`

4. ✅ **Ensure port uniqueness** - Each lidar uses different ports to avoid conflicts

---

## 🔍 Troubleshooting

### ❌ LiDAR Not Detected

- ✅ Verify IP address matches the configuration and QR code
- 🔌 Check physical connections (Ethernet cables, power supply)
- 🌐 Check network connectivity:
  ```bash
  ping 192.168.1.10
  ping 192.168.1.18
  ```

### 🚫 Connection Issues

- 🔥 Ensure no firewall is blocking the configured ports
- 📄 Verify the `user_config_path` in `livox_params.yaml` points to `multiple_test.json`
- 🔄 Try restarting the LiDARs (power cycle)

### 📊 No Point Cloud Data

- 📡 Check if the ROS2 topic is publishing:
  ```bash
  ros2 topic list
  ros2 topic echo /livox/lidar
  ```
- ⚙️ Verify `publish_freq` parameter is set correctly
- 🔍 Check ROS2 logs for errors:
  ```bash
  ros2 launch livox_ros_driver2 multiple_lidars.launch.py --log-level debug
  ```

---

## 📚 Additional Resources

- 📖 [Livox SDK2 Documentation](https://github.com/Livox-SDK/Livox-SDK2)
- 🌐 [Livox Official Website](https://www.livoxtech.com/)
- 💬 [ROS2 Documentation](https://docs.ros.org/en/humble/)

---

**Happy LiDAR Scanning!** 🎉📡
