import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pcl
from pcl import pointcloud2_to_array, array_to_pointcloud2

class VoxelGridFilterNode(Node):

    def __init__(self):
        super().__init__('voxel_grid_filter_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/input_pointcloud',
            self.pointcloud_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/downsampled_pointcloud', 10)
        self.voxel_size = 0.05  # Set your voxel size here (e.g., 0.05 meters)

    def pointcloud_callback(self, msg):
        # Convert ROS PointCloud2 message to PCL PointCloud
        cloud_array = pointcloud2_to_array(msg)
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_array(cloud_array)

        # Apply Voxel Grid filter
        voxel_filter = pcl.filters.VoxelGrid()  # Adjusted to the correct filter initialization
        voxel_filter.set_input_cloud(pcl_cloud)
        voxel_filter.set_leaf_size(self.voxel_size, self.voxel_size, self.voxel_size)

        downsampled_cloud = voxel_filter.filter()

        # Convert the filtered PCL cloud back to ROS PointCloud2 message
        downsampled_msg = array_to_pointcloud2(downsampled_cloud.to_array(), msg.header.frame_id)
        self.publisher.publish(downsampled_msg)

def main(args=None):
    rclpy.init(args=args)
    voxel_grid_filter_node = VoxelGridFilterNode()
    rclpy.spin(voxel_grid_filter_node)
    voxel_grid_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
