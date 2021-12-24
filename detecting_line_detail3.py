#! /usr/bin/env python3
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import numpy as np
import threading

bridge = CvBridge()
def scan(data):
    global cmd_vel

    laser_range = data.ranges[0:10]
    laser_arr = np.array(laser_range)
    no_hindrance = np.count_nonzero(laser_arr)
    result = np.count_nonzero(laser_arr >= 0.2 )

    if result > 0 or no_hindrance == 10 :
        # MOVE
        cmd_vel.linear.x = 0.1
        pass
    else :
        # STOP
        cmd_vel.linear.x = 0.0
    pub.publish(cmd_vel)

    # rospy.spin()  
    pass


def callback(frame):
    global cmd_vel

    if frame != None:
        cv_image = bridge.imgmsg_to_cv2 (frame, 'bgr8')
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        # cv_image = cv.rotate(cv_image, cv.ROTATE_180 )
        print('뒤 :', cv_image[326:330, 356:360])
        print('앞 :', cv_image[326:330, 436:440])

        ar =  np.mean(cv_image[326:330, 356:360])
        print('test : ', ar)
        # 중앙 사각형 검정색이면. 존나 삽질했는데 알고보니 선생님이 알려준 이 조건문이 틀렸다.
        if np.all(cv_image[326:330, 356:360]) < 70 or np.all(cv_image[326:330, 436:440]) < 70:
            cmd_vel.linear.x = 0.1
        # 왼쪽 위 사각형 검정이면
        elif np.all(cv_image[246:250, 356:360]) < 70:
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.1
        # 오른쪽 위 사각형 검정이면
        elif np.all(cv_image[406:410, 356:360]) < 70:
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = -0.1
        # 왼쪽 아래 사각형 검정색이면
        elif np.all(cv_image[86:90, 436:440]) < 70 or np.all(cv_image[166:170, 436:440]) < 70 or np.all(cv_image[246:250, 436:440]) < 70:
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.1
        # 오른쪽 아래사각형 검정색
        elif np.all(cv_image[406:410, 436:440]) < 70 or np.all(cv_image[486:490, 436:440]) < 70 or np.all(cv_image[566:570, 436:440]) < 70:
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = -0.1
        # 중앙 아래 사각형이 밖으로 나가면 정지
        elif np.all(cv_image[326:330, 436:440]) > 70 and np.all(cv_image[326:330, 436:440]) > 120 :
            cmd_vel.linear.x = 0.0
        else:
            pass

        pub.publish(cmd_vel)
    else:
        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)
        print('카메라 연결 안됐음')
    return


# def subscrb(name, data, func):
#     while True:
#         rospy.Subscriber(name, data, func)
#     pass


def subscrb(name, data, func):
    rospy.Subscriber(name, data, func)
    rospy.spin()
    pass

def subscrb2(name, data, func):
    rospy.Subscriber(name, data, func)
    rospy.spin()
    pass


if __name__ == '__main__':
    rospy.init_node('img_cv_node')
    cmd_vel = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    first = threading.Thread(target=subscrb, args=('/scan',LaserScan, scan))
    second = threading.Thread(target=subscrb2, args=('/cv_camera/image_raw',Image, callback))
    first.start()
    second.start()
    first.join()
    second.join()
    # rospy.spin()

