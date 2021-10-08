#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

# Global Variables

g_current_orientation = 0.0
g_desired_orientation = math.pi
robot_state = 1
vel_value = Twist()
pub_cmd_vel = None

def laser_scan_cb(msg):

    laser_reading = msg.ranges[0]

    take_action(laser_reading)

def take_action(distance):

    global vel_value, pub_cmd_vel

    if distance >= 1:
        vel_value.linear.x = 0.1

    else:
        vel_value.linear.x = 0.0

    pub_cmd_vel.publish(vel_value)


def odmetry_cb(msg):
    msg


def main():

    global pub_cmd_vel
    
    rospy.init_node('turn_robot')

    sub_laser = rospy.Subscriber('/scan', LaserScan, laser_scan_cb)
    sub_odom = rospy.Subscriber('/odom', Odometry, odmetry_cb)

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
