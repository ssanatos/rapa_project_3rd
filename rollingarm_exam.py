#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64

# pub /left_arm_controller/command std_msgs/Float64 "data: 1.0"
def main():
    rospy.init_node("rollingarm")
    left_arm_pub = rospy.Publisher('/left_arm_controller/command', Float64, queue_size=10)
    right_arm_pub = rospy.Publisher('/right_arm_controller/command', Float64, queue_size=10)

    while not rospy.is_shutdown():
        direction = input("direction(w,x,s,e,a,d) : ")
        # velocity = float(input("velocity(0.0 ~ 2.0) : "))
        
        if direction == 'w':
            # left_arm_pub.publish(-velocity)
            # right_arm_pub.publish(-velocity)
            left_arm_pub.publish(-1)
            right_arm_pub.publish(-1)
        elif direction == 'x':
            # left_arm_pub.publish(velocity)
            # right_arm_pub.publish(velocity)
            left_arm_pub.publish(1)
            right_arm_pub.publish(1)
        elif direction == 's':
            left_arm_pub.publish(0.0)
            right_arm_pub.publish(0.0)
        elif direction == 'e':
            # left_arm_pub.publish(-velocity)
            # right_arm_pub.publish(velocity)
            left_arm_pub.publish(-1)
            right_arm_pub.publish(1)
        elif direction == 'a':
            # left_arm_pub.publish(velocity)
            # right_arm_pub.publish(velocity)
            right_arm_pub.publish(1)           
        elif direction == 'd':
            # left_arm_pub.publish(velocity)
            left_arm_pub.publish(1)
            # right_arm_pub.publish(velocity)
        else:
            continue
        
        rospy.sleep(0.01)
    return

if __name__ == "__main__":
    main()
    pass