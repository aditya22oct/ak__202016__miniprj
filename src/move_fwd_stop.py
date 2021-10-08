#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

# Global Variables

g_current_orientation = 0.0
g_desired_orientation = 0.0
g_ang_precision = 0.001
robot_state = 1
vel_value = Twist()
pub_cmd_vel = None

def laser_scan_cb(msg):

    laser_reading = msg.ranges[0]
    take_action(laser_reading)


def odmetry_cb(msg):

    global g_current_orientation

    curr_quat_x = msg.pose.pose.orientation.x
    curr_quat_y = msg.pose.pose.orientation.y
    curr_quat_z = msg.pose.pose.orientation.z
    curr_quat_w = msg.pose.pose.orientation.w

    curr_euler = euler_from_quaternion([curr_quat_x, curr_quat_y, curr_quat_z, curr_quat_w])
    g_current_orientation = curr_euler[2]

    rospy.loginfo(g_current_orientation)


def take_action(distance):

    global vel_value, pub_cmd_vel, g_current_orientation, g_desired_orientation
    global g_ang_precision

    if distance >= 1:
        vel_value.linear.x = 0.1
    else:
        vel_value.linear.x = 0.0

    ang_err = g_desired_orientation - g_current_orientation

    if abs(ang_err) > g_ang_precision:
        vel_value.angular.z = ang_err
    else:
        vel_value.angular.z = 0.0

    pub_cmd_vel.publish(vel_value)


def main():

    global pub_cmd_vel

    rospy.init_node('turn_robot')

    sub_laser = rospy.Subscriber('/scan', LaserScan, laser_scan_cb)
    sub_odom = rospy.Subscriber('/odom', Odometry, odmetry_cb)

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
