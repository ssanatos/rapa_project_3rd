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

    pass


def callback(frame):
    global cmd_vel

    if frame != None:
        cv_image = bridge.imgmsg_to_cv2 (frame, 'bgr8')
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        cv_image = cv.rotate(cv_image, cv.ROTATE_180)
        # 이미지 사이즈 = 640 x 480   478x480 으로 봐야되. 이 사이즈를 잡아늘리나봐.
        # 검정 라인이 있는 영역은 평균값이 더 적을 거라는 가정하에.
        img_list =[]
        img_list.append(np.mean(cv_image[0:53, 450:480]))
        img_list.append(np.mean(cv_image[53:106, 450:480]))   
        img_list.append(np.mean(cv_image[106:159, 450:480]))
        img_list.append(np.mean(cv_image[159:212, 450:480]))
        img_list.append(np.mean(cv_image[212:265, 450:480]))  # center
        img_list.append(np.mean(cv_image[265:318, 450:480]))
        img_list.append(np.mean(cv_image[318:371, 450:480]))
        img_list.append(np.mean(cv_image[371:424, 450:480]))
        img_list.append(np.mean(cv_image[424:478, 450:480]))
        # 윗라인
        forward_list = []
        forward_list.append(np.mean(cv_image[159:212, 420:450]))
        forward_list.append(np.mean(cv_image[212:265, 420:450]))  # center
        forward_list.append(np.mean(cv_image[265:318, 420:450]))

        # img_list.append(np.mean(cv_image[41:71, 450:480]))
        # img_list.append(np.mean(cv_image[71:142, 450:480]))   
        # img_list.append(np.mean(cv_image[142:213, 450:480]))
        # img_list.append(np.mean(cv_image[213:284, 450:480]))
        # img_list.append(np.mean(cv_image[284:355, 450:480]))  # center
        # img_list.append(np.mean(cv_image[355:426, 450:480]))
        # img_list.append(np.mean(cv_image[426:478, 450:480]))
        # img_list.append(np.mean(cv_image[478:600, 450:480]))
        # # 윗라인
        # forward_list = []
        # forward_list.append(np.mean(cv_image[213:284, 420:450]))
        # forward_list.append(np.mean(cv_image[284:355, 420:450]))  # center
        # forward_list.append(np.mean(cv_image[355:426, 420:450]))


        tmp = min(img_list)
        index = img_list.index(tmp)
        print("밑 : ",img_list)
        print(index)

        tmp2 = min(forward_list)
        index2 = forward_list.index(tmp2)
        print("위 : ",forward_list)
        print(index2)

        if tmp2 < 110 :
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = (1-index2)/10
            pub.publish(cmd_vel)
    
        else :
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = (4-index)/10
            pub.publish(cmd_vel)
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

