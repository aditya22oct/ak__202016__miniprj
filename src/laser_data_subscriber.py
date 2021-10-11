#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

regions = None

g_regions = {}
def LaserScan_cb(msg):

    global g_regions
    laser_data = list(msg.ranges)
    front_region = laser_data[0:18] + laser_data[342:]
    left_front_region = laser_data[18:54]
    right_front_region = laser_data[306:342]
    print(len(laser_data))
    print(len(front_region))
    print(len(left_front_region))
    print(len(right_front_region ))
    print(type(laser_data))
    g_regions = {'front_region': min([min(front_region),10]),
    'left_front_region': min([min(left_front_region),10]),
    'right_front_region': min([min(right_front_region),10])
    }
    print(g_regions)

def find_gap():
    global regions
    print(regions)


def main():
    rospy.init_node('laser_subscriber')
    rospy.Subscriber('/scan', LaserScan, LaserScan_cb)
    rospy.spin()


if __name__ == '__main__':
    main()
