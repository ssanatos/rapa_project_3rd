#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import numpy as np

def callback(data):
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.1
    pub.publish(cmd_vel)
    laser_range = data.ranges[0:10]
    # print('laser_range : ',laser_range)
    laser_arr = np.array(laser_range)
    result = np.count_nonzero(laser_arr >= 0.2 )# or laser_arr == 0.0)
    # cmd_vel = Twist()
    if result > 0:
        # MOVE
        cmd_vel.linear.x = 0.1
    else :
        # STOP
        cmd_vel.linear.x = 0.0
    pub.publish(cmd_vel)
    pass

if __name__ == '__main__':
    rospy.init_node('parking')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    # sub = rospy.Subscriber('/cv_camera/image_raw',Image,callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()
