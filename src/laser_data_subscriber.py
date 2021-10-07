#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def LaserScan_cb(msg):
    print(len(msg.ranges))

rospy.init_node('laser_subscriber')
rospy.Subscriber('/scan', LaserScan, LaserScan_cb)
rospy.spin()
