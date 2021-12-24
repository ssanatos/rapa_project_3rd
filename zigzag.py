#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
#PI = 3.1415926535897
toRAD = 0.017453292519943295

def rotate():
    # Starts a new node
    rospy.init_node('zigzag', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()

    print("start zigzag !")
    rate = rospy.Rate(3)   
    term_zigzag = 10

    msg.linear.x = 0.1
    angular_z = 0.1
    while not rospy.is_shutdown():
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

if __name__ == '__main__':
    try:
        rotate()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
