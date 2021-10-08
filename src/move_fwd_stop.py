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
g_curr_laser_reading = 10.0
vel_value = Twist()
pub_cmd_vel = None
kill_node = False
g_bot_state= 0


def laser_scan_cb(msg):

    global g_curr_laser_reading

    g_curr_laser_reading = msg.ranges[0]




def odmetry_cb(msg):

    global g_current_orientation

    curr_quat_x = msg.pose.pose.orientation.x
    curr_quat_y = msg.pose.pose.orientation.y
    curr_quat_z = msg.pose.pose.orientation.z
    curr_quat_w = msg.pose.pose.orientation.w

    curr_quat = [curr_quat_x, curr_quat_y, curr_quat_z, curr_quat_w]
    curr_euler = euler_from_quaternion(curr_quat)
    g_current_orientation = curr_euler[2]


def move_forward():

    global vel_value, g_bot_state

    if g_curr_laser_reading >= 1:
        vel_value.linear.x = 0.1
    else:
        vel_value.linear.x = 0.0
        g_bot_state = 1
        rospy.loginfo("Wall detected")

    ang_err = g_desired_orientation - g_current_orientation

    if abs(ang_err) > g_ang_precision:
        vel_value.angular.z = ang_err
    else:
        vel_value.angular.z = 0.0


def stop():

    global vel_value

    vel_value.linear.x = 0.0
    vel_value.angular.z = 0.0
    rospy.loginfo("Stopping")


def main():

    global pub_cmd_vel, vel_value, kill_node

    rospy.init_node('turn_robot')
    r = rospy.Rate(10)

    sub_laser = rospy.Subscriber('/scan', LaserScan, laser_scan_cb)
    sub_odom = rospy.Subscriber('/odom', Odometry, odmetry_cb)

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    while not kill_node:
        if g_bot_state == 0:
            move_forward()
            rospy.loginfo("Moving forward")
        elif g_bot_state == 1:
            stop()
            kill_node = True
        pub_cmd_vel.publish(vel_value)
        r.sleep()

    rospy.loginfo("Finished")

if __name__ == '__main__':
    main()
