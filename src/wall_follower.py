#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Global Variables
g_laser_readings = {}
g_dist = 1 # Distance from wall
g_robot_states = {
    0: "Finding Wall",
    1: "Turning Left",
    2: "Following wall"
    }
g_robot_curr_state = 0
g_vel_value = Twist()


def laser_scan_cb(msg):

    global g_regions
    laser_data = list(msg.ranges)
    front_region = laser_data[0:18] + laser_data[342:]
    front_left_region = laser_data[18:54]
    front_right_region = laser_data[306:342]

    g_laser_readings = {
        'front_region': min([min(front_region),10]),
        'front_left_region': min([min(front_left_region),10]),
        'front_right_region': min([min(front_right_region),10])
        }


def robot_state_evaluator():

    f_region = g_laser_readings['front_region']
    fl_region = g_laser_readings['front_left_region']
    fr_region = g_laser_readings['front_right_region']

    #f<d fr < d fl < d action: turn_left c 1
    #f<d fr < d fl > d action: turn left c 1
    #f<d fr > d fl < d action: turn left c 1
    #f<d fr > d fl > d action: turn_left c 1
    #f>d fr < d fl < d action: find wall c 0
    #f>d fr < d fl > d action: follow wall c 2
    #f>d fr > d fl < d action: find Wall c 0
    #f>d fr > d fl > d action: find wall c 0


def find_gap():
    global regions
    print(regions)

def main():
    rospy.init_node('laser_subscriber')
    sub_laser_scan =rospy.Subscriber('/scan', LaserScan, laser_scan_cb)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        robot_state_evaluator()

        if g_robot_curr_state == 0:
            find_wall()
        elif g_robot_curr_state == 1:
            turn_left()
        elif g_robot_curr_state == 2:
            follow_wall()
        else:
            pass

        pub_cmd_vel.publish(g_vel_value)
        rate.sleep()


if __name__ == '__main__':
    main()
