#! /usr/bin/env python3
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import numpy as np

bridge = CvBridge()
def scan(data):
    global cmd_vel
    # 라이다 코드 함수로 넣기.
    laser_range = data.ranges[0:10]
    laser_arr = np.array(laser_range)
    no_hindrance = np.count_nonzero(laser_arr)
    result = np.count_nonzero(laser_arr >= 0.2 )

    if result > 0 or no_hindrance == 10 :
        # MOVE
        cmd_vel.linear.x = 0.1            
    else :
        # STOP
        cmd_vel.linear.x = 0.0
    pub.publish(cmd_vel)
    pass


def callback(frame):
    global cmd_vel

    if frame != None:
        cv_image = bridge.imgmsg_to_cv2 (frame, 'bgr8')
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

        # 중앙 사각형 검정색이면
        if np.all(cv_image[320:330, 350:360]) < 70 or np.all(cv_image[320:330, 430:440]) < 70:
            cmd_vel.linear.x = 0.1
        # 왼쪽 위 사각형 검정이면
        elif np.all(cv_image[240:250, 350:360]) < 70:
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.1
        # 오른쪽 위 사각형 검정이면
        elif np.all(cv_image[400:410, 350:360]) < 70:
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = -0.1
        # 왼쪽 아래 사각형 검정색이면
        elif np.all(cv_image[80:90, 430:440]) < 70 or np.all(cv_image[160:170, 430:440]) < 70 or np.all(cv_image[240:250, 430:440]) < 70:
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.1
        # 오른쪽 아래사각형 검정색
        elif np.all(cv_image[400:410, 430:440]) < 70 or np.all(cv_image[480:490, 430:440]) < 70 or np.all(cv_image[560:570, 430:440]) < 70:
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = -0.1
        # 중앙 아래 사각형이 밖으로 나가면 정지
        elif np.all(cv_image[320:330, 430:440]) > 70 and np.all(cv_image[320:330, 430:440]) < 120 :
            cmd_vel.linear.x = 0.0
        else:
            pass

        pub.publish(cmd_vel)
    else:
        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)
        print('카메라 연결 안됐음')
    return


if __name__ == '__main__':
    rospy.init_node('img_cv_node')
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.1
    sub = rospy.Subscriber('/scan', LaserScan, scan)
    sub2 = rospy.Subscriber('/cv_camera/image_raw',Image,callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()

