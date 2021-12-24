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

    if result > 0 or no_hindrance == 0 :
        # MOVE
        cmd_vel.linear.x = 0.1
        pass
    else :
        # STOP
        cmd_vel.linear.x = 0.0
    # pub.publish(cmd_vel)

    # rospy.spin()  
    pass


def callback(frame):
    global cmd_vel

    if frame != None:
        cv_image = bridge.imgmsg_to_cv2 (frame, 'bgr8')
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        cv_image = np.clip(((1 + 2) * cv_image - 128 * 2), 0, 255)
        # cv_image = cv.rotate(cv_image, cv.ROTATE_180 )
        # print('뒤 :', cv_image[326:330, 356:360])
        # print('앞 :', cv_image[326:330, 436:440])
        # numm = np.array(cv_image) cv_image의 사이즈는 480x640인데 자꾸 right22, right23이 안나온다. 메모리 문제인가.

        center_1 =  np.mean(cv_image[326:330, 356:360])
        center_2 =  np.mean(cv_image[326:330, 436:440])
        left_1 =  np.mean(cv_image[246:250, 356:360])
        right_1 =  np.mean(cv_image[406:410, 356:360])
        left_21 =  np.mean(cv_image[86:90, 436:440])
        left_22 =  np.mean(cv_image[166:170, 436:440])
        left_23 =  np.mean(cv_image[246:250, 436:440])
        right_21 =  np.mean(cv_image[406:410, 436:440])
        right_22 =  np.mean(cv_image[486:490, 436:440])
        right_23 =  np.mean(cv_image[566:570, 436:440])

        print("center_1  : ", center_1)
        print("center_2  : ", center_2)
        print("left_1  : ", left_1)
        print("right_1  : ", right_1)
        print("left_21  : ", left_21)
        print("left_22  : ", left_22)
        print("left_23  : ", left_23)
        print("right_21  : ", right_21)
        print("right_22  : ", right_22) # 두개가 null값이 나온다.
        print("right_23  : ", right_23) # 두개가 null값이 나온다. 왤까.


        # 중앙 사각형 검정색이면.
        if center_1 < 70 or center_2 < 70:
            print("중앙")
            cmd_vel.linear.x = 0.1
            pub.publish(cmd_vel)
        # 왼쪽 위 사각형 검정이면
        elif left_1 < 70:
            print("좌상")
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.28
            pub.publish(cmd_vel)
        # 오른쪽 위 사각형 검정이면
        elif right_1 < 70:
            print("우상")
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = -0.28
            pub.publish(cmd_vel)
        # 왼쪽 아래 사각형 검정색이면
        elif left_21 < 70 or left_22 < 70 or left_23 < 70:
            print("좌하")
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.28
            pub.publish(cmd_vel)
        # 오른쪽 아래사각형 검정색
        elif right_21 < 70 or right_22 < 70 or right_23 < 70:
            print("우하")
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = -0.28
            pub.publish(cmd_vel)
        # 중앙 아래 사각형이 밖으로 나가면 정지
        elif center_2 > 140 :
            print("밖")
            cmd_vel.linear.x = 0.05
            # cmd_vel.linear.z = -0.01
            pub.publish(cmd_vel)
        else:
            cmd_vel.linear.x = 0.01
            pub.publish(cmd_vel)
            pass

        # pub.publish(cmd_vel)
    else:
        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)
        print('카메라 연결 안됐음')
    return


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

