import rospy
import math
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('rotate')
    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    msg = Twist()
    relative_angle = math.radians(90)  # 목표각도
    angular_speed = 1.0                     # 미는 힘
    duration = relative_angle / angular_speed
    time2end = rospy.Time.now() + rospy.Duration(duration)
    # print(time2end )

    msg.angular.z = angular_speed
    while rospy.Time.now() < time2end:
        pub.publish(msg)
        rospy.sleep(0.01)

    msg.angular.z = 0.0
    pub.publish(msg)
    pass

if __name__ == "__main__":
    main()
    pass