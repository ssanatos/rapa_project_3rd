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
        # 이미지 사이즈 = 640 x 480
        # 검정 라인이 있는 영역은 평균값이 더 적을 거라는 가정하에.
        img_list =[]
        img_list.append(np.mean(cv_image[0:160, 0:120]))
        img_list.append(np.mean(cv_image[160:320, 0:120]))   
        img_list.append(np.mean(cv_image[320:480, 0:120]))
        img_list.append(np.mean(cv_image[480:640, 0:120]))

        img_list.append(np.mean(cv_image[0:160, 120:240]))
        img_list.append(np.mean(cv_image[160:320, 120:240]))   
        img_list.append(np.mean(cv_image[320:480, 120:240]))
        img_list.append(np.mean(cv_image[480:640, 120:240]))

        img_list.append(np.mean(cv_image[0:160, 240:360]))
        img_list.append(np.mean(cv_image[160:320, 240:360]))   
        img_list.append(np.mean(cv_image[320:480, 240:360]))
        img_list.append(np.mean(cv_image[480:640, 240:360]))

        img_list.append(np.mean(cv_image[0:160, 360:480]))
        img_list.append(np.mean(cv_image[160:320, 360:480]))   
        img_list.append(np.mean(cv_image[320:480, 360:480]))
        img_list.append(np.mean(cv_image[480:640, 360:480]))
        
        print(img_list)

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

