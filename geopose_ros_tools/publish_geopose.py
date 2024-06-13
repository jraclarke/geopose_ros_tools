#!/usr/bin/env python
import os

import pandas as pd
from rclpy.time import Time
import rclpy
from rclpy.node import Node
from ogc_geopose_msgs.msg import GeoPoseBasicYPRStamped
from rclpy.logging import get_logger
from ament_index_python.packages import get_package_share_directory

class OGCGeoposePublisher(Node):

    def __init__(self):
        super().__init__('ogc_geopose_publisher')
        self.publisher_ = self.create_publisher(msg_type=GeoPoseBasicYPRStamped, topic='ogc_geopose', qos_profile=10)
        self.rate = 100  # 100Hz from example input file.
        self.time_period = 1./100
        self.r_factor = 100 / self.rate # Used later for iters calc.
        self.loop_rate = self.create_rate(self.rate, self.get_clock())
        self.logger = get_logger(__name__)
        self.df = pd.read_csv(os.path.join(get_package_share_directory('geopose_ros_tools'), 'resource',
                                      'STREETDRONE.INS.2023-04-25T113136.000.csv'))
        self.timer = self.create_timer(self.time_period, self.run_publisher)
        self.iters = self.df.shape[0] / self.r_factor
        self.i = 5000

    def populate_geopose_ypr(self, iter, geopose_ypr_stamped):
        ros_time = + (self.df['Time (GPS µs)'][iter] / 1000000) + 315964782
        seconds = int(ros_time)
        nanoseconds = int((ros_time - seconds) * 1e9)
        timestamp = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()

        geopose_ypr_stamped.header.stamp = timestamp
        geopose_ypr_stamped.header.frame_id = "map"
        geopose_ypr_stamped.geoposebasicypr.angles.yaw = self.df['Heading (deg)'][iter]
        geopose_ypr_stamped.geoposebasicypr.angles.pitch = self.df['Pitch (deg)'][iter]
        geopose_ypr_stamped.geoposebasicypr.angles.roll = self.df['Roll (deg)'][iter]
        geopose_ypr_stamped.geoposebasicypr.position.lat = self.df['Latitude (deg)'][iter]
        geopose_ypr_stamped.geoposebasicypr.position.lon = self.df['Longitude (deg)'][iter]
        geopose_ypr_stamped.geoposebasicypr.position.h = self.df['Altitude (m)'][iter]

        return geopose_ypr_stamped


    def run_publisher(self):
        geopose_ypr_stamped = GeoPoseBasicYPRStamped()

        if self.i < self.iters:
            geopose_ypr_stamped = self.populate_geopose_ypr(self.i * self.r_factor, geopose_ypr_stamped)
            self.publisher_.publish(geopose_ypr_stamped)

        self.i += 1
def main(args=None):
    rclpy.init(args=args)

    ogc_geopose_publisher_node = OGCGeoposePublisher()
    rclpy.spin(ogc_geopose_publisher_node)

    ogc_geopose_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
