#!/usr/bin/env python3
"""
LiDAR TF Calibration Node for Dual MID-360 Setup

This node publishes static transforms for the dual Livox MID-360 configuration
to visualize and verify the extrinsic calibration in RViz.

Frame hierarchy:
  base_link (root)
  ├── lidar_L1 (left sensor, 11cm to left, facing forward)
  └── lidar_L2 (right sensor, 11cm to right, facing backward)

Author: Generated for FAST-LIO dual sensor calibration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math


class LidarTFCalibration(Node):
    """
    Publishes static TF transforms for dual MID-360 LiDAR calibration.
    """

    def __init__(self):
        super().__init__('lidar_tf_calibration')

        # Create static transform broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Declare parameters for easy calibration adjustment
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('l1_frame', 'lidar_L1')
        self.declare_parameter('l2_frame', 'lidar_L2')

        # L1 (Left sensor) extrinsics - facing forward
        self.declare_parameter('l1_x', 0.0)      # meters
        self.declare_parameter('l1_y', 0.11)     # 11cm to the left
        self.declare_parameter('l1_z', 0.0)      # meters
        self.declare_parameter('l1_roll', -90.0) # degrees (Z-axis points LEFT/outward)
        self.declare_parameter('l1_pitch', 0.0)  # degrees
        self.declare_parameter('l1_yaw', 0.0)    # degrees (facing forward)

        # L2 (Right sensor) extrinsics - facing backward
        self.declare_parameter('l2_x', 0.0)      # meters
        self.declare_parameter('l2_y', -0.11)    # 11cm to the right
        self.declare_parameter('l2_z', 0.0)      # meters
        self.declare_parameter('l2_roll', -90.0) # degrees (Z-axis points RIGHT/outward)
        self.declare_parameter('l2_pitch', 0.0)  # degrees
        self.declare_parameter('l2_yaw', 180.0)  # degrees (facing backward)

        # Get parameter values
        base_frame = self.get_parameter('base_frame').value
        l1_frame = self.get_parameter('l1_frame').value
        l2_frame = self.get_parameter('l2_frame').value

        # Publish static transforms
        transforms = []

        # Transform: base_link -> lidar_L1
        transforms.append(self.create_transform(
            parent_frame=base_frame,
            child_frame=l1_frame,
            x=self.get_parameter('l1_x').value,
            y=self.get_parameter('l1_y').value,
            z=self.get_parameter('l1_z').value,
            roll=self.get_parameter('l1_roll').value,
            pitch=self.get_parameter('l1_pitch').value,
            yaw=self.get_parameter('l1_yaw').value
        ))

        # Transform: base_link -> lidar_L2
        transforms.append(self.create_transform(
            parent_frame=base_frame,
            child_frame=l2_frame,
            x=self.get_parameter('l2_x').value,
            y=self.get_parameter('l2_y').value,
            z=self.get_parameter('l2_z').value,
            roll=self.get_parameter('l2_roll').value,
            pitch=self.get_parameter('l2_pitch').value,
            yaw=self.get_parameter('l2_yaw').value
        ))

        # Broadcast all transforms
        self.tf_static_broadcaster.sendTransform(transforms)

        self.get_logger().info('=' * 60)
        self.get_logger().info('LiDAR TF Calibration Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Base frame: {base_frame}')
        self.get_logger().info('')
        self.get_logger().info('L1 (Left sensor - facing forward):')
        self.get_logger().info(f'  Frame: {l1_frame}')
        self.get_logger().info(f'  Position: [{self.get_parameter("l1_x").value:.3f}, '
                              f'{self.get_parameter("l1_y").value:.3f}, '
                              f'{self.get_parameter("l1_z").value:.3f}] m')
        self.get_logger().info(f'  Rotation: Roll={self.get_parameter("l1_roll").value}°, '
                              f'Pitch={self.get_parameter("l1_pitch").value}°, '
                              f'Yaw={self.get_parameter("l1_yaw").value}°')
        self.get_logger().info('')
        self.get_logger().info('L2 (Right sensor - facing backward):')
        self.get_logger().info(f'  Frame: {l2_frame}')
        self.get_logger().info(f'  Position: [{self.get_parameter("l2_x").value:.3f}, '
                              f'{self.get_parameter("l2_y").value:.3f}, '
                              f'{self.get_parameter("l2_z").value:.3f}] m')
        self.get_logger().info(f'  Rotation: Roll={self.get_parameter("l2_roll").value}°, '
                              f'Pitch={self.get_parameter("l2_pitch").value}°, '
                              f'Yaw={self.get_parameter("l2_yaw").value}°')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Transforms published. Visualize in RViz with:')
        self.get_logger().info('  - Fixed Frame: base_link')
        self.get_logger().info('  - Add TF display to see coordinate frames')
        self.get_logger().info('=' * 60)

    def create_transform(self, parent_frame, child_frame, x, y, z, roll, pitch, yaw):
        """
        Create a TransformStamped message from position and euler angles.

        Args:
            parent_frame: Parent frame ID
            child_frame: Child frame ID
            x, y, z: Translation in meters
            roll, pitch, yaw: Rotation in degrees (will be converted to quaternion)

        Returns:
            TransformStamped message
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # Set translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # Convert euler angles (degrees) to quaternion
        # Using ZYX convention (Yaw-Pitch-Roll)
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        # Calculate quaternion components
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy

        return t


def main(args=None):
    rclpy.init(args=args)
    node = LidarTFCalibration()

    try:
        # Keep node alive to maintain static transforms
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
