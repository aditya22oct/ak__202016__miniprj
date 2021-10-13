#!/usr/bin/env python3

import rospy
from ak_202016_prj.srv import RotateMsg, RotateMsgResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

pub_cmd_vel = None

def srv_cb(request):

    vel_value = Twist()
    srv_response = RotateMsgResponse()

    vel_value.linear.x = 0.0
    vel_value.angular.z = 0.0

    pub_cmd_vel.publish(vel_value)

    srv_response.success = True
    
    rospy.loginfo('Robot Stopped')

    return srv_response

def main():

    global pub_cmd_vel

    rospy.init_node('stop_rotate_robot_service_server')
    stop_rotate_robot_service = rospy.Service('/rotate_now',RotateMsg, srv_cb)
    rospy.loginfo('Server for service stopping and rotating robot started !')
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.spin()


if __name__ == '__main__':
    main()
