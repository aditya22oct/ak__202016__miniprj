#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

# Declaring Global Variables

g_desired_pos = Point()
g_desired_pos.x = 2.0
g_desired_pos.y = 1.0
g_desired_pos.z = 0.0

g_odometry_obj = Odometry()
g_current_orientation = 0.0
g_current_pose = g_odometry_obj.pose.pose.position

g_pose_acc = 0.1
g_orientation_acc = 0.0175

g_vel_value = Twist()
g_ang_vel_z = 0.05
g_lin_vel_x = 0.1

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



def change_state(state):

    global g_robot_curr_state

    g_robot_curr_state = state
    rospy.loginfo(g_robot_states[g_robot_curr_state])


def ang_vel_setter(curr_orientation_diff):

    global g_vel_value

    if abs(curr_orientation_diff) > g_orientation_acc:
        if curr_orientation_diff > 0:
            g_vel_value.angular.z = g_lin_vel_x
            g_vel_value.linear.x = 0.0
        else:
            g_vel_value.angular.z = -g_lin_vel_x
            g_vel_value.linear.x = 0.0
    else:
        g_vel_value.angular.z = 0.0
        g_vel_value.linear.x = 0.0
        rospy.loginfo('Orientation_set')
        change_state(1)


def adjust_orientation():

    # condition 180 orientation
    if g_desired_pos.y == 0 and g_desired_pos.x < 0:
        # rospy.loginfo("In 180 condition")
        desired_orietation = math.pi

        if g_current_orientation < 0:
            # rospy.loginfo("In 180 if condition")
            current_orientation = 360 + g_current_orientation
            curr_orientation_diff = desired_orietation - current_orientation
            ang_vel_setter(curr_orientation_diff)

        else:
            # rospy.loginfo("In 180 else condition")
            curr_orientation_diff = desired_orietation - g_current_orientation
            ang_vel_setter(curr_orientation_diff)

    else:
        # rospy.loginfo("In else condition")
        x_diff = g_desired_pos.x - g_current_pose.x
        y_diff = g_desired_pos.y - g_current_pose.y

        desired_orietation = math.atan2(y_diff, x_diff)

        curr_orientation_diff = desired_orietation - g_current_orientation
        ang_vel_setter(curr_orientation_diff)


def check_orientation(curr_orientation_diff):

    if abs(curr_orientation_diff) > g_orientation_acc:
        return_var = True

    else:
        return_var = False

    return return_var


def drive_towards_goal():

    global g_vel_value

    x_diff = g_desired_pos.x - g_current_pose.x
    y_diff = g_desired_pos.y - g_current_pose.y

    distance = math.sqrt(math.pow(x_diff, 2)+math.pow(y_diff, 2))
    desired_orietation = math.atan2(y_diff, x_diff)
    curr_orientation_diff = desired_orietation - g_current_orientation

    if distance > g_pose_acc:
        rospy.loginfo(distance)
        g_vel_value.linear.x = g_lin_vel_x
        g_vel_value.angular.z = 0
    else:
        g_vel_value.linear.x = 0.0
        g_vel_value.angular.z = 0
        rospy.loginfo('Goal Reached')
        change_state(2)

    if check_orientation(curr_orientation_diff):
        rospy.loginfo(curr_orientation_diff)
        change_state(0)


def stop():

    global g_vel_value

    g_vel_value.linear.x = 0
    g_vel_value.angular.z = 0


def main():

    global g_kill_node

    rospy.init_node('go_to_point_w-o_obstacle')
    r = rospy.Rate(4)
    sub_odom = rospy.Subscriber('/odom', Odometry, odmetry_cb)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while not g_kill_node:
        if g_robot_curr_state == 0:
            adjust_orientation()
        elif g_robot_curr_state == 1:
            drive_towards_goal()
        elif g_robot_curr_state == 2:
            stop()
            g_kill_node = True

        pub_cmd_vel.publish(g_vel_value)
        r.sleep()


if __name__ == '__main__':
    main()
