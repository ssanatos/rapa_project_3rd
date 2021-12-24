#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64

# pub /left_arm_controller/command std_msgs/Float64 "data: 1.0"
def main():
    rospy.init_node("rollingarm")
    left_arm_pub = rospy.Publisher('/left_arm_controller/command', Float64, queue_size=10)
    # left_direction =1.0
    while not rospy.is_shutdown():
        left_direction = float(input("direction(-1.0~1.0) : "))
        left_arm_pub.publish(left_direction)
        rospy.sleep(0.01)
    return

if __name__ == "__main__":
    main()
    pass