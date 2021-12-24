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
        # 이정도 나눈것도 너무 성기다.
        # 32 pixel 간격으로 읽어들이기
        # 30:34 - 606:610  ,   350:354  -  446:450      (4행 20열.)
        # l101 = np.mean(cv_image[30:34, 350:354])
        # l102 = np.mean(cv_image[62:66, 350:354])
        # l103 = np.mean(cv_image[94:98, 350:354])
        # l104 = np.mean(cv_image[126:130, 350:354])
        # l105 = np.mean(cv_image[158:162, 350:354])
        # l106 = np.mean(cv_image[190:194, 350:354])
        # l107 = np.mean(cv_image[222:226, 350:354])
        # l108 = np.mean(cv_image[254:258, 350:354])
        # l109 = np.mean(cv_image[94:, 350:354])
        # l110 = np.mean(cv_image[94:98, 350:354])
        # l111 = np.mean(cv_image[94:98, 350:354])
        # l112 = np.mean(cv_image[94:98, 350:354])
        # l113 = np.mean(cv_image[94:98, 350:354])
        # l114 = np.mean(cv_image[94:98, 350:354])
        # l115 = np.mean(cv_image[94:98, 350:354])
        # l116 = np.mean(cv_image[94:98, 350:354])
        # l117 = np.mean(cv_image[94:98, 350:354])
        # l118 = np.mean(cv_image[94:98, 350:354])
        # l119 = np.mean(cv_image[94:98, 350:354])
        # l120 = np.mean(cv_image[94:98, 350:354])

        # 2차원 넘파이 어레이를 만들어서 컬러의 평균값 넣기. 그 평균값으로 if문 비교계산하거나
        # collect = np.empty(shape=(4,20))
        # for idxi, i in enumerate(range(354,480,32)):
        #     for idxk, k in enumerate(range(2,640,32)):
        #         collect[idxi,idxk] = np.mean(cv_image[k-4:k, i-4:i])
        
        # 위의 어레이보다 1행, 1열 적은 넘파이 어레이 만들고, 옆의 값 끼리 평균값의 차에 가중치를 곱해서 x,z값 구하기.
        # collect = np.empty(shape=(3,19))
        # for idxi, i in enumerate(range(354,448,32)):
        #     for idxk, k in enumerate(range(2,608,32)):
        #         row_gap = abs(np.mean(cv_image[k-4:k, i-4:i]) - np.mean(cv_image[k+28:k+32, i-4:i]))
        #         col_gap = abs(np.mean(cv_image[k-4:k, i-4:i]) - np.mean(cv_image[k-4:k, i+28:i+32]))
        #         if row_gap > 30 or col_gap > 30
        #             collect[idxi,idxk] = 

        # 각 평균값의 차이에 가중치를 곱해서 다 더해
        collect = np.empty(shape=(3,19))
        for idxi, i in enumerate(range(354,448,32)):
            for idxk, k in enumerate(range(2,608,32)):
                row_gap = abs(np.mean(cv_image[k-4:k, i-4:i]) - np.mean(cv_image[k+28:k+32, i-4:i]))
                col_gap = abs(np.mean(cv_image[k-4:k, i-4:i]) - np.mean(cv_image[k-4:k, i+28:i+32]))
                collect[idxi,idxk] = (row_gap + col_gap)*((9.5-idxk)*(idxi+1)**2)
        cmd_vel.angular.z = sum(collect)*0.00018797

        



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

