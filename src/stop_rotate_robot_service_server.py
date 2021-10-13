#!/usr/bin/env python3

import rospy
from ak_202016_prj.srv import RotateMsg, RotateMsgResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

pub_cmd_vel = None
sub_odom = None
vel_value = Twist()

odom_call_after_stop =  False
robot_orientation_after_stop = None
robot_orientation_final = None
g_current_orientation = None
g_orientation_acc = 0.05


def odmetry_cb(msg):

    global first_odom_call, robot_orientation_start, g_current_orientation

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

    if odom_call_after_stop:
        robot_orientation_after_stop = curr_euler[2] * (180 / math.pi)
        rospy.loginfo('Robot start orientation saved')
        rospy.loginfo(robot_orientation_after_stop)
        odom_call_after_stop = False
    else:
        g_current_orientation = curr_euler[2] * (180 / math.pi)
        rospy.loginfo('Robot current orientation')
        rospy.loginfo(g_current_orientation)


def stop_robot():

    global vel_value, odom_call_after_stop

    vel_value.linear.x = 0.0
    vel_value.angular.z = 0.0

    pub_cmd_vel.publish(vel_value)

    odom_call_after_stop = True


def robot_orientation_final_setter(dir_rotation, degrees):

    global robot_orientation_final

    # clockwise
    if dir_rotation:
        robot_orientation_final = robot_orientation_after_stop - degrees

    # counter-clockwise
    elif dir_rotation:
        robot_orientation_final = robot_orientation_after_stop + degrees


def srv_cb(request):

    stop_robot()
    robot_orientation_final_setter(request.clockwise, request.degrees)

    srv_response = RotateMsgResponse()

    if


    srv_response.success = True

    rospy.loginfo('Robot Stopped')

    return srv_response

def main():

    global pub_cmd_vel, sub_odom

    rospy.init_node('stop_rotate_robot_service_server')
    stop_rotate_robot_service = rospy.Service('/rotate_now',RotateMsg, srv_cb)
    rospy.loginfo('Server for service stopping and rotating robot started !')
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    sub_odom = rospy.Subscriber('/odom', Odometry, odmetry_cb)
    rospy.spin()


if __name__ == '__main__':
    main()
