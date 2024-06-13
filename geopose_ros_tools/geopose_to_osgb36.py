#!/usr/bin/env python
import pyproj
import tf2_ros
from tf_transformations import quaternion_from_euler
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from ogc_geopose_msgs.msg import GeoPoseBasicYPRStamped
import time
from math import pi
from rclpy.logging import get_logger

class GeoposeGnssBngConv(Node):

    def __init__(self):
        super().__init__('gnss_bng_conv')
        self.bng_publisher_ = self.create_publisher(PoseStamped, 'ogc_geopose_bng', 10)
        self.wgs84_subscriber = self.create_subscription(
            msg_type=GeoPoseBasicYPRStamped,
            topic='/ogc_geopose',
            callback=self.fix_callback,
            qos_profile=10)
        self.rate = 100  # 100Hz from example input file.
        self.last_alt = 0
        self.transformer = pyproj.Transformer.from_crs(4326, 27700)
        self.i = 0
        self.geopose_bng = PoseStamped()
        self.geopose_bng.header.frame_id = "geopose"
        self.last_z = 0
        self.logger = get_logger(__name__)

    def coord_transform(self, x, y, z):
        # Optimised for repeat transformations.
        x_out, y_out, z_out = self.transformer.transform(y, x, z)

        return x_out, y_out, z_out

    def fix_callback(self, msg):
        # Reduce to 10 Hz
        # if (self.i % 1 == 0):

        # self.geopose_bng.header.stamp = msg.header.stamp
        self.geopose_bng.header.stamp = self.get_clock().now().to_msg()
        start = time.time()
        x, y, z = self.coord_transform(msg.geoposebasicypr.position.lon,
                                       msg.geoposebasicypr.position.lat, msg.geoposebasicypr.position.h)
        end = time.time()
        # print('time elapsed = {0}'.format(end-start))
        # Correct for Geoid (OSGM15)
        z = z - 46

        self.geopose_bng.header.frame_id = "map"
        self.geopose_bng.pose.position.x = x
        self.geopose_bng.pose.position.y = y
        self.geopose_bng.pose.position.z = z
        roll = msg.geoposebasicypr.angles.roll * (pi / 180)
        pitch = msg.geoposebasicypr.angles.pitch * (pi / 180)
        yaw = msg.geoposebasicypr.angles.yaw * (pi / 180)
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        self.geopose_bng.pose.orientation.x = quaternion[0]
        self.geopose_bng.pose.orientation.y = quaternion[1]
        self.geopose_bng.pose.orientation.z = quaternion[2]
        self.geopose_bng.pose.orientation.w = -quaternion[3]
        self.handle_geopose_bng(x, y, z)
        #self.logger.info("{},{},{},{},{},{},{},{},{}".format(msg.geoposebasicypr.position.lat,
        #                                                     msg.geoposebasicypr.position.lon,
        #                                                     msg.geoposebasicypr.position.h, x, y, z, roll, pitch, yaw))
        self.i += 1
        self.bng_publisher_.publish(self.geopose_bng)

    def handle_geopose_bng(self, x, y, z):
        br = tf2_ros.TransformBroadcaster(self)
        q = quaternion_from_euler(0, 0, 0)
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'ogc_geopose'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    geopose_gnss_bng_conv_node = GeoposeGnssBngConv()
    rclpy.spin(geopose_gnss_bng_conv_node)
    geopose_gnss_bng_conv_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


