#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64, String


def main():
    rospy.init_node('my_little_nod_py')
    left_arm = rospy.Publisher('left_arm_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        direction = input("direction : ")

        if direction == 's':    # stop    
            speed_msg = 0.0
        elif direction == 'a':   # left
            speed_msg = 1.0
        elif direction == 'd':   # right
            speed_msg = 1.0
        else :
            speed_msg = 0.0

        left_arm.publish(speed_msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
