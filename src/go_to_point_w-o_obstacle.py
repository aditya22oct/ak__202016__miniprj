#! usr/bin/#!/usr/bin/env python3

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
