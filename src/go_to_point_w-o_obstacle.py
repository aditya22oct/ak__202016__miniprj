#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

# Declaring Global Variables

g_desired_pos = Point()
g_desired_pos.x = 5.0
g_desired_pos.y = 0.0
g_desired_pos.z = 0.0


g_kill_node = False
g_bot_state= 0
g_pose_acc = 0.01

def odmetry_cb(msg):

    curr_pose = msg.pose.pose.position
    take_action(curr_pose)


def take_action(pose_data):

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel_value = Twist()

    x_diff = g_desired_pos.x - pose_data.x
    y_diff = g_desired_pos.y - pose_data.y

    distance = math.sqrt(math.pow(x_diff, 2)+math.pow(y_diff, 2))

    if distance >= g_pose_acc:
        vel_value.linear.x = 0.2
    else:
        vel_value.linear.x = 0.0

    pub_cmd_vel.publish(vel_value)
    rospy.loginfo(distance)


def main():

    global pub_cmd_vel, vel_value, kill_node

    rospy.init_node('turn_robot')
    r = rospy.Rate(10)

    sub_odom = rospy.Subscriber('/odom', Odometry, odmetry_cb)

    rospy.spin()


if __name__ == '__main__':
    main()
