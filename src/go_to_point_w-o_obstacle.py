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

g_current_orientation = None
g_current_pose = None

g_pose_acc = 0.1
g_orientation_acc = 0.1

g_vel_value = Twist()

g_kill_node = False
g_robot_curr_state = 0
g_robot_states= {0: 'Adjusting Orientation',
              1: 'Moving towards goal',
              2: 'Reached goal'
              }


def odmetry_cb(msg):

    global g_current_orientation, g_current_pose

    g_current_pose = msg.pose.pose.position

    curr_quat_x = msg.pose.pose.orientation.x
    curr_quat_y = msg.pose.pose.orientation.y
    curr_quat_z = msg.pose.pose.orientation.z
    curr_quat_w = msg.pose.pose.orientation.w

    curr_quat = [curr_quat_x,
                 curr_quat_y,
                 curr_quat_z,
                 curr_quat_w
                 ]

    curr_euler = euler_from_quaternion(curr_quat)
    g_current_orientation = curr_euler[2]


def adjust_orientation():

    global g_vel_value

    x_diff = g_desired_pos.x - g_current_pose.x
    y_diff = g_desired_pos.y - g_current_pose.y

    desired_orietation = math.atan2(y_diff, x_diff)

    curr_orientation_diff = desired_orietation - g_current_orientation

    if abs(curr_orientation_diff) > g_orientation_acc:
        if curr_orientation_diff > 0:
            g_vel_value.angular.z = 0.1
        else:
            g_vel_value.angular.z = -0.1
    else:
        g_vel_value.angular.z = 0.0


def drive_towards_goal():

    global g_vel_value

    x_diff = g_desired_pos.x - g_current_pose.x
    y_diff = g_desired_pos.y - g_current_pose.y

    distance = math.sqrt(math.pow(x_diff, 2)+math.pow(y_diff, 2))

    if distance > g_pose_acc:
        vel_value.linear.x = 0.2
    else:
        vel_value.linear.x = 0.0

def take_action(pose_data):

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_value = Twist()

    x_diff = g_desired_pos.x - pose_data.x
    y_diff = g_desired_pos.y - pose_data.y

    distance = math.sqrt(math.pow(x_diff, 2)+math.pow(y_diff, 2))

    if distance > g_pose_acc:
        vel_value.linear.x = 0.2
    else:
        vel_value.linear.x = 0.0
    rospy.loginfo(vel_value.linear.x)
    pub_cmd_vel.publish(vel_value)
    rospy.loginfo(distance)


def main():

    global pub_cmd_vel, vel_value, kill_node

    rospy.init_node('go_to_point_w-o_obstacle')
    r = rospy.Rate(4)
    sub_odom = rospy.Subscriber('/odom', Odometry, odmetry_cb)

    rospy.spin()


if __name__ == '__main__':
    main()
