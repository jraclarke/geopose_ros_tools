#!/usr/bin/env python
import pandas as pd

import rospy
from geopose_msgs.msg import GeoPoseBasicYPRStamped

rospy.init_node('publish_geopose')
df = pd.read_csv('/home/jclarke/GeoPose/STREETDRONE.INS.2023-04-25T113136.000.csv')
pub = rospy.Publisher('geopose', GeoPoseBasicYPRStamped, queue_size=10)
geopose_ypr_stamped = GeoPoseBasicYPRStamped()

rate = 100  # Hz
# file is at 100Hz
r_factor = 100 / rate
r = rospy.Rate(rate)


def populate_geopose_ypr(iter):
    ros_time = + (df['Time (GPS µs)'][iter] / 1000000) + 315964782
    timestamp = rospy.Time.from_sec(ros_time)

    geopose_ypr_stamped.header.stamp = timestamp
    geopose_ypr_stamped.header.frame_id = "map"
    geopose_ypr_stamped.geoposebasicypr.angles.yaw = df['Heading (deg)'][iter]
    geopose_ypr_stamped.geoposebasicypr.angles.pitch = df['Pitch (deg)'][iter]
    geopose_ypr_stamped.geoposebasicypr.angles.roll = df['Roll (deg)'][iter]
    geopose_ypr_stamped.geoposebasicypr.position.lat = df['Latitude (deg)'][iter]
    geopose_ypr_stamped.geoposebasicypr.position.lon = df['Longitude (deg)'][iter]
    geopose_ypr_stamped.geoposebasicypr.position.h = df['Altitude (m)'][iter]

    return geopose_ypr_stamped


def run_publisher():
    i = 5000
    iters = df.shape[0] / r_factor

    while i < iters:
        geopose_ypr_stamped = populate_geopose_ypr(i * r_factor)
        pub.publish(geopose_ypr_stamped)
        r.sleep()

        i += 1


if __name__ == '__main__':
    run_publisher()
