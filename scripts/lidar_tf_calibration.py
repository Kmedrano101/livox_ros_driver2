#!/usr/bin/env python3
"""
LiDAR TF Calibration Node for Dual MID-360 Setup

This node publishes static transforms for the dual Livox MID-360 configuration
to visualize and verify the extrinsic calibration in RViz.

Additionally, it subscribes to the raw lidar topics and republishes them with
the correct frame_id to match the TF tree for proper visualization in RViz.

Frame hierarchy:
  base_link (root)
  ├── lidar_L1 (left sensor, 11cm to left, facing forward)
  └── lidar_L2 (right sensor, 11cm to right, facing backward)

Author: Generated for FAST-LIO dual sensor calibration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster, TransformException, Buffer, TransformListener
import math
import numpy as np
from sensor_msgs_py import point_cloud2
import struct


class LidarTFCalibration(Node):
    """
    Publishes static TF transforms for dual MID-360 LiDAR calibration.
    """

    def __init__(self):
        super().__init__('lidar_tf_calibration')

        # Create static transform broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Create TF buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declare parameters for easy calibration adjustment
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('l1_frame', 'lidar_L1')
        self.declare_parameter('l2_frame', 'lidar_L2')

        # Declare parameters for lidar topics
        # When using single_mid360.launch.py with multi_topic=0, topic is '/livox/lidar'
        # When using multiple_lidars.launch.py with multi_topic=1, topics are '/livox/lidar_192_168_1_XX'
        self.declare_parameter('l1_topic', '/livox/lidar_192_168_1_10')
        self.declare_parameter('l2_topic', '/livox/lidar_192_168_1_18')

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
        self.declare_parameter('l2_roll', 90.0)  # degrees (Z-axis points RIGHT/outward)
        self.declare_parameter('l2_pitch', 0.0)  # degrees
        self.declare_parameter('l2_yaw', 180.0)  # degrees (facing backward)

        # Get parameter values
        base_frame = self.get_parameter('base_frame').value
        self.l1_frame = self.get_parameter('l1_frame').value
        self.l2_frame = self.get_parameter('l2_frame').value
        l1_topic = self.get_parameter('l1_topic').value
        l2_topic = self.get_parameter('l2_topic').value

        # Publish static transforms
        transforms = []

        # Transform: base_link -> lidar_L1
        transforms.append(self.create_transform(
            parent_frame=base_frame,
            child_frame=self.l1_frame,
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
            child_frame=self.l2_frame,
            x=self.get_parameter('l2_x').value,
            y=self.get_parameter('l2_y').value,
            z=self.get_parameter('l2_z').value,
            roll=self.get_parameter('l2_roll').value,
            pitch=self.get_parameter('l2_pitch').value,
            yaw=self.get_parameter('l2_yaw').value
        ))

        # Broadcast all transforms
        self.tf_static_broadcaster.sendTransform(transforms)

        # Create subscribers for lidar point clouds
        self.l1_sub = self.create_subscription(
            PointCloud2,
            l1_topic,
            self.l1_callback,
            10
        )

        self.l2_sub = self.create_subscription(
            PointCloud2,
            l2_topic,
            self.l2_callback,
            10
        )

        # Create publishers for republished point clouds with correct frame_id
        self.l1_pub = self.create_publisher(
            PointCloud2,
            '/lidar_L1/pointcloud',
            10
        )

        self.l2_pub = self.create_publisher(
            PointCloud2,
            '/lidar_L2/pointcloud',
            10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('Dual LiDAR TF Calibration Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Base frame: {base_frame}')
        self.get_logger().info('')
        self.get_logger().info('L1 (Left sensor - facing forward):')
        self.get_logger().info(f'  Frame: {self.l1_frame}')
        self.get_logger().info(f'  Input Topic: {l1_topic}')
        self.get_logger().info(f'  Output Topic: /lidar_L1/pointcloud')
        self.get_logger().info(f'  Position: [{self.get_parameter("l1_x").value:.3f}, '
                              f'{self.get_parameter("l1_y").value:.3f}, '
                              f'{self.get_parameter("l1_z").value:.3f}] m')
        self.get_logger().info(f'  Rotation: Roll={self.get_parameter("l1_roll").value}°, '
                              f'Pitch={self.get_parameter("l1_pitch").value}°, '
                              f'Yaw={self.get_parameter("l1_yaw").value}°')
        self.get_logger().info('')
        self.get_logger().info('L2 (Right sensor - facing backward):')
        self.get_logger().info(f'  Frame: {self.l2_frame}')
        self.get_logger().info(f'  Input Topic: {l2_topic}')
        self.get_logger().info(f'  Output Topic: /lidar_L2/pointcloud')
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
        self.get_logger().info('  - Subscribe to /lidar_L1/pointcloud and /lidar_L2/pointcloud')
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

    def transform_point_cloud(self, cloud_msg, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0,
                             trans_x=0.0, trans_y=0.0, trans_z=0.0):
        """
        Transform a point cloud by applying translation and rotation.

        Args:
            cloud_msg: Input PointCloud2 message
            roll_deg: Roll rotation in degrees
            pitch_deg: Pitch rotation in degrees
            yaw_deg: Yaw rotation in degrees
            trans_x, trans_y, trans_z: Translation in meters (applied before rotation)

        Returns:
            Transformed PointCloud2 message
        """
        # Convert angles to radians
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        # Create rotation matrix (ZYX convention)
        # Rz(yaw) * Ry(pitch) * Rx(roll)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cr = math.cos(roll)
        sr = math.sin(roll)

        # Combined rotation matrix
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr          ]
        ])

        # Read points from cloud
        points_list = []
        for point in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        if len(points_list) == 0:
            return cloud_msg  # Return original if no valid points

        # Convert to numpy array for transformation
        points = np.array(points_list)

        # To undo driver transform (which does: R * p + t):
        # We need to: R^(-1) * (p - t)
        # Step 1: Subtract translation (undo driver's translation)
        translation = np.array([trans_x, trans_y, trans_z])
        points_translated = points - translation

        # Step 2: Apply inverse rotation
        transformed_points = np.dot(points_translated, R.T)

        # Create new point cloud message
        transformed_cloud = PointCloud2()
        transformed_cloud.header = cloud_msg.header
        transformed_cloud.height = cloud_msg.height
        transformed_cloud.width = cloud_msg.width
        transformed_cloud.fields = cloud_msg.fields
        transformed_cloud.is_bigendian = cloud_msg.is_bigendian
        transformed_cloud.point_step = cloud_msg.point_step
        transformed_cloud.row_step = cloud_msg.row_step
        transformed_cloud.is_dense = cloud_msg.is_dense

        # Read all point data including additional fields (intensity, etc.)
        all_points = []
        field_names = [field.name for field in cloud_msg.fields]

        for i, point_data in enumerate(point_cloud2.read_points(cloud_msg, field_names=field_names, skip_nans=False)):
            if i < len(transformed_points):
                # Create new point with transformed xyz and original other fields
                new_point = list(point_data)
                new_point[0] = transformed_points[i, 0]  # x
                new_point[1] = transformed_points[i, 1]  # y
                new_point[2] = transformed_points[i, 2]  # z
                all_points.append(tuple(new_point))
            else:
                all_points.append(point_data)

        # Create new point cloud with transformed points
        transformed_cloud = point_cloud2.create_cloud(transformed_cloud.header, cloud_msg.fields, all_points)

        return transformed_cloud

    def l1_callback(self, msg):
        """
        Callback for L1 lidar point cloud.
        The Livox driver has already applied extrinsic transformation (roll=-90°, translation=[0,110mm,0]).
        We need to apply the inverse transformation to get back to sensor frame,
        then publish with frame_id=lidar_L1 so RViz/TF can apply our TF transform.
        """
         try:
             # DEBUG: Log original frame_id and first few points
             self.get_logger().info(f'L1 INPUT - frame_id: {msg.header.frame_id}, points: {msg.width * msg.height}',
                                    throttle_duration_sec=2.0)

             # Read first point for debugging
             first_points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
             if len(first_points) > 0:
                 self.get_logger().info(f'L1 BEFORE transform - First point: x={first_points[0][0]:.3f}, y={first_points[0][1]:.3f}, z={first_points[0][2]:.3f}',
                                       throttle_duration_sec=2.0)

             # Driver extrinsic: roll=-90°, yaw=0°, translation=[0, 110mm, 0]
             # Driver applies: p_out = R * p_sensor + t
             # To undo: p_sensor = R^(-1) * (p_out - t)
             # Step 1: Subtract driver translation [0, 0.110, 0]
             # Step 2: Apply inverse rotation (roll=+90°, yaw=0°)

             transformed_cloud = self.transform_point_cloud(
                 msg,
                 roll_deg=90.0,    # Inverse of driver's -90°
                 pitch_deg=0.0,
                 yaw_deg=0.0,      # Inverse of driver's 0°
                 trans_x=0.0,      # Driver's translation to subtract
                 trans_y=0.110,    # Driver's 110mm offset to subtract
                 trans_z=0.0
             )

             # DEBUG: Log after transformation
             after_points = list(point_cloud2.read_points(transformed_cloud, field_names=("x", "y", "z"), skip_nans=True))
             if len(after_points) > 0:
                 self.get_logger().info(f'L1 AFTER transform - First point: x={after_points[0][0]:.3f}, y={after_points[0][1]:.3f}, z={after_points[0][2]:.3f}',
                                       throttle_duration_sec=2.0)

             # Set frame_id to match TF tree
             transformed_cloud.header.frame_id = self.l1_frame
             transformed_cloud.header.stamp = msg.header.stamp

             self.get_logger().info(f'L1 OUTPUT - frame_id: {transformed_cloud.header.frame_id}',
                                   throttle_duration_sec=2.0)

             # Republish
             self.l1_pub.publish(transformed_cloud)

         except Exception as e:
             self.get_logger().error(f'L1 transformation error: {str(e)}')

    def l2_callback(self, msg):
        """
        Callback for L2 lidar point cloud.
        The Livox driver has already applied extrinsic transformation (roll=-90°, yaw=180°).
        We need to apply the inverse transformation to get back to sensor frame,
        then publish with frame_id=lidar_L2 so RViz/TF can apply our TF transform.
        """
        try:
            # DEBUG: Log original frame_id and first few points
            self.get_logger().info(f'L2 INPUT - frame_id: {msg.header.frame_id}, points: {msg.width * msg.height}',
                                   throttle_duration_sec=2.0)

            # Read first point for debugging
            first_points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if len(first_points) > 0:
                self.get_logger().info(f'L2 BEFORE transform - First point: x={first_points[0][0]:.3f}, y={first_points[0][1]:.3f}, z={first_points[0][2]:.3f}',
                                      throttle_duration_sec=2.0)

            # Driver extrinsic: roll=+90°, yaw=180°, translation=[0, -110mm, 0]
            # Driver applies: p_out = R * p_sensor + t
            # To undo: p_sensor = R^(-1) * (p_out - t)
            # Step 1: Subtract driver translation [0, -0.110, 0]
            # Step 2: Apply inverse rotation (roll=-90°, yaw=-180°)

            transformed_cloud = self.transform_point_cloud(
                msg,
                roll_deg=-90.0,   # Inverse of driver's +90°
                pitch_deg=0.0,
                yaw_deg=-180.0,   # Inverse of driver's +180°
                trans_x=0.0,      # Driver's translation to subtract
                trans_y=-0.110,   # Driver's -110mm offset to subtract
                trans_z=0.0
            )

            # DEBUG: Log after transformation
            after_points = list(point_cloud2.read_points(transformed_cloud, field_names=("x", "y", "z"), skip_nans=True))
            if len(after_points) > 0:
                self.get_logger().info(f'L2 AFTER transform - First point: x={after_points[0][0]:.3f}, y={after_points[0][1]:.3f}, z={after_points[0][2]:.3f}',
                                      throttle_duration_sec=2.0)

            # Set frame_id to match TF tree
            transformed_cloud.header.frame_id = self.l2_frame
            transformed_cloud.header.stamp = msg.header.stamp

            self.get_logger().info(f'L2 OUTPUT - frame_id: {transformed_cloud.header.frame_id}',
                                  throttle_duration_sec=2.0)

            # Republish
            self.l2_pub.publish(transformed_cloud)

        except Exception as e:
            self.get_logger().error(f'L2 transformation error: {str(e)}')


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
