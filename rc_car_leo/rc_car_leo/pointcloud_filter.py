import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pcl
import numpy as np
import std_msgs
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('point_cloud_filter')
        qos_profile = QoSProfile(
            depth=10,  # Queue size
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Set to BEST_EFFORT
        )
        self.subscriber = self.create_subscription(PointCloud2, 
                                                '/stereo/points', 
                                                self.point_cloud_callback, 
                                                qos_profile)
        self.filtered_publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)

    def point_cloud_callback(self, msg):
        # Convert ROS PointCloud2 message to numpy array
        point_cloud_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        point_cloud_np = np.array(list(point_cloud_data))

        # Create a PCL PointCloud
        cloud = pcl.PointCloud() # -> Generating error
        cloud.from_array(point_cloud_np.astype(np.float32))

        # Apply a voxel grid filter
        voxel_filter = cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(0.1, 0.1, 0.1)  # Adjust the leaf size as needed
        filtered_cloud = voxel_filter.filter()

        # Convert filtered cloud back to ROS PointCloud2 message
        filtered_msg = self.pcl_to_ros(filtered_cloud)
        self.filtered_publisher.publish(filtered_msg)

    def pcl_to_ros(self, pcl_cloud):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"  # Change to your frame_id

        # Get the points from PCL
        points = pcl_cloud.to_array()

        # Create a PointCloud2 message
        return pc2.create_cloud_xyz32(header, points[:, 0], points[:, 1], points[:, 2])

def main(args=None):
    rclpy.init(args=args)
    point_cloud_filter = PointCloudFilter()
    rclpy.spin(point_cloud_filter)
    point_cloud_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
