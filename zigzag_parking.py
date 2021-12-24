#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
#PI = 3.1415926535897
# toRAD = 0.017453292519943295

def rotate():
    # Starts a new node
    rospy.init_node('zigzag', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()

    # print("start zigzag !")
    rate = rospy.Rate(3)   
    term_zigzag = 10

    msg.linear.x = 0.1
    angular_z = 0.1
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('/scan', LaserScan, callback)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        time2end = 0
        msg.angular.z = angular_z
        while time2end < term_zigzag:
            pub.publish(msg)
            rate.sleep()
            time2end = time2end + 1

        time2end = 0
        msg.angular.z = angular_z * -1
        while time2end < term_zigzag:
            pub.publish(msg)
            rate.sleep()
            time2end = time2end + 1


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
        cmd_vel.linear.x = 0.0
    # pub.publish(cmd_vel)
    pass


if __name__ == '__main__':
    try:
        rotate()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
