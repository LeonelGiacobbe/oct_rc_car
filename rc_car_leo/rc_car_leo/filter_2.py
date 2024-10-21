# pointcloud_filter/pointcloud_filter_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filter_node')
        qos_profile = QoSProfile(
            depth=10,  # Queue size
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Set to BEST_EFFORT
        )
        # Subscribers
        self.subscription = self.create_subscription(
            PointCloud2,
            'stereo/points',
            self.listener_callback,
            qos_profile)
        
        # Publisher
        self.publisher = self.create_publisher(PointCloud2, 'stereo/points/filtered', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Received point cloud data')

        # Unpack the PointCloud2 data
        width = msg.width
        height = msg.height
        point_step = msg.point_step
        row_step = msg.row_step
        is_dense = msg.is_dense

        # Extract the data from the PointCloud2 message
        points = np.frombuffer(msg.data, dtype=np.float32)
        points = points.reshape((height, width, -1))  # Reshape to (height, width, channels)

        # Apply a simple filter (e.g., keep points where z > 0)
        filtered_points = points[points[..., 2] > 0]  # Keep points with z > 0

        # Create a new PointCloud2 message
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = 1  # 1 for unorganized
        filtered_msg.width = filtered_points.shape[0]
        filtered_msg.fields = msg.fields
        filtered_msg.is_bigendian = msg.is_bigendian
        filtered_msg.point_step = msg.point_step
        filtered_msg.row_step = filtered_msg.point_step * filtered_msg.width
        filtered_msg.is_dense = is_dense
        filtered_msg.data = filtered_points.tobytes()  # Convert back to bytes

        # Publish the filtered point cloud
        self.publisher.publish(filtered_msg)
        self.get_logger().info('Published filtered point cloud')

def main(args=None):
    rclpy.init(args=args)
    pointcloud_filter_node = PointCloudFilterNode()
    rclpy.spin(pointcloud_filter_node)
    pointcloud_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
