#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

regions = None

def LaserScan_cb(msg):

    global regions
    laser_data = msg.ranges
    regions = [
    laser_data[0:29],
    laser_data[330:359],
    laser_data[270:299],
    laser_data[240:269],
    ]


def find_gap():
    global regions
    print(regions)


def main():
    rospy.init_node('laser_subscriber')
    rospy.Subscriber('/scan', LaserScan, LaserScan_cb)
    find_gap()
    rospy.spin()


if __name__ == '__main__':
    main()
