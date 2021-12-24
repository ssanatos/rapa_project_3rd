#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

def rotate(cmd_vel):
    relative_angle = math.radians(90)        # 목표각도
    angular_speed = 1.0                      # 미는 힘
    duration = relative_angle / angular_speed
    cmd_vel.angular.z = angular_speed
    time2end = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < time2end:
        pub.publish(cmd_vel)
        rospy.sleep(0.01)
    pass


def callback(data):
    laser_range = data.ranges[0:10]
    laser_arr = np.array(laser_range)
    result = np.count_nonzero(laser_arr >= 0.2 )
    cmd_vel = Twist()
    if result > 0:
        # MOVE
        cmd_vel.linear.x = 0.1
        pub.publish(cmd_vel)
    else :
        # ROTATE
        rotate(cmd_vel)
    cmd_vel.angular.z = 1.0  # 0.0
    pub.publish(cmd_vel)
    pass

if __name__ == '__main__':
    rospy.init_node('parking')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()



