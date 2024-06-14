"""
Generate and publish a PointCloud2 msg from a source ply file.
Lightly adapted from https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo.
"""
import os
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import numpy as np
import open3d as o3d
from ament_index_python.packages import get_package_share_directory

class PC2Publisher(Node):

    def __init__(self):
        super().__init__('pcd_publisher_node')
        ply_path = os.path.join(get_package_share_directory('geopose_ros_tools'), 'resource',
                                      'STREETDRONE.POINTCLOUD_BNG.2024-05-03T171508.000.ply')

        # Use Open3D to read point clouds and meshes.
        pcd = o3d.io.read_point_cloud(ply_path)
        self.points = np.asarray(pcd.points)
        print(self.points.shape)

        self.pc2_publisher = self.create_publisher(sensor_msgs.PointCloud2, 'pointcloud', 10)
        period = 1
        self.timer = self.create_timer(period, self.timer_callback)


    def timer_callback(self):
        # Convert the numpy array into a sensor_msgs.PointCloud2 object.
        self.stamp=self.get_clock().now().to_msg()
        self.pcd = np_arr_to_pc2(self.points, 'map', self.stamp)

        self.pc2_publisher.publish(self.pcd)
        rclpy.spin_once(self)


def np_arr_to_pc2(points, parent_frame, stamp):
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    header = std_msgs.Header(frame_id=parent_frame, stamp=stamp)
    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


def main(args=None):
    rclpy.init(args=args)
    pc2_publisher = PC2Publisher()
    rclpy.spin(pc2_publisher)

    pc2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()