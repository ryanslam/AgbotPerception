import rclpy
from rclpy.node import Node
import ros2_numpy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import open3d as o3d
import numpy as np

class AgbotLidarListener(Node):

    def __init__(self):
        super().__init__('agbot_lidar_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/quanergy/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()

    def listener_callback(self, msg):
        xyz = ros2_numpy.point_cloud2.point_cloud2_to_array(msg)

        print(xyz['xyz'].shape)

        # Create an Open3D point cloud
        point_cloud = o3d.geometry.PointCloud()
        self.vis.add_geometry(point_cloud)

        point_cloud.points = o3d.utility.Vector3dVector(xyz['xyz'])

        # Update the visualization
        self.vis.update_geometry(point_cloud)
        self.vis.poll_events()
        self.vis.update_renderer()



def main(args=None):
    rclpy.init(args=args)

    agbot_lidar_subscriber = AgbotLidarListener()

    rclpy.spin(agbot_lidar_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    agbot_lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()