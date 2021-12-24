#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

def rotate():
    cmd_vel = Twist()
    relative_angle = math.radians(90)  # 목표각도
    angular_speed = 1.0                     # 미는 힘
    duration = relative_angle / angular_speed
    time2end = rospy.Time.now() + rospy.Duration(duration)

    cmd_vel.angular.z = angular_speed
    while rospy.Time.now() < time2end:
        pub.publish(cmd_vel)
        rospy.sleep(0.01)

    cmd_vel.angular.z = 0.0
    pub.publish(cmd_vel)
    pass


def callback(data):
    laser_range = data.ranges[0:10]
    # print('laser_range : ',laser_range)
    laser_arr = np.array(laser_range)
    result = np.count_nonzero(laser_arr >= 0.2 )# or laser_arr == 0.0)
    cmd_vel = Twist()
    if result > 0:
        # MOVE
        cmd_vel.linear.x = 0.1
    else :
        # STOP
        rotate()
    pub.publish(cmd_vel)
    pass

if __name__ == '__main__':
    rospy.init_node('parking')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()
