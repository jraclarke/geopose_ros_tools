#!/usr/bin/env python
import rospy
import time
import sys
import pyproj
import tf
import math
from geometry_msgs.msg import PoseStamped
from geopose_msgs.msg import GeoPoseBasicYPRStamped
import time
from math import pi


class Gnss_bng_conv(object):

    def __init__(self):
        self.last_alt = 0
        self.transformer = pyproj.Transformer.from_crs(4326, 27700)
        self.i = 0
        self.geopose_bng = PoseStamped()
        self.geopose_bng.header.frame_id = "geopose"
        self.last_z = 0

    def coord_transform(self, x, y, z):
        # Optimised for repeat transformations.
        x_out, y_out, z_out = self.transformer.transform(y, x, z)

        return x_out, y_out, z_out

    def fix_callback(self, msg):
        # Reduce to 10 Hz
        # if (self.i % 1 == 0):

        # self.geopose_bng.header.stamp = msg.header.stamp
        self.geopose_bng.header.stamp = rospy.Time.now()
        start = time.time()
        x, y, z = self.coord_transform(msg.geoposebasicypr.position.lon,
                                       msg.geoposebasicypr.position.lat, msg.geoposebasicypr.position.h)
        end = time.time()
        # print('time elapsed = {0}'.format(end-start))
        # Correct for Geoid
        z = z - 46

        self.geopose_bng.header.frame_id = "map"
        self.geopose_bng.pose.position.x = x
        self.geopose_bng.pose.position.y = y
        self.geopose_bng.pose.position.z = z
        roll = msg.geoposebasicypr.angles.roll * (pi / 180)
        pitch = msg.geoposebasicypr.angles.pitch * (pi / 180)
        yaw = msg.geoposebasicypr.angles.yaw * (pi / 180)
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.geopose_bng.pose.orientation.x = quaternion[0]
        self.geopose_bng.pose.orientation.y = quaternion[1]
        self.geopose_bng.pose.orientation.z = quaternion[2]
        self.geopose_bng.pose.orientation.w = -quaternion[3]
        self.handle_geopose_bng(x, y, z)
        # rospy.loginfo("{},{},{},{},{},{},{},{},{}".format(msg.position.lat, msg.position.lon, msg.position.h,
        #                                         x, y, z, roll, pitch, yaw))
        # self.i += 1

    def handle_geopose_bng(self, x, y, z):
        br = tf.TransformBroadcaster()
        br.sendTransform((x, y, z),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "geopose",
                         "map")

    def run(self):
        rate = 100
        rospy.init_node('gnss_bng_conv', anonymous=True)
        loop_rate = rospy.Rate(rate)  # 10hz
        rospy.loginfo("Publishing /geopose_bng at {0} Hz.".format(rate))
        rospy.Subscriber("/geopose", GeoPoseBasicYPRStamped, self.fix_callback)

        pub_geopose_bng = rospy.Publisher('geopose_bng', PoseStamped, queue_size=20)

        while not rospy.is_shutdown():
            pub_geopose_bng.publish(self.geopose_bng)
            loop_rate.sleep()


if __name__ == '__main__':
    gbc = Gnss_bng_conv()
    gbc.run()
