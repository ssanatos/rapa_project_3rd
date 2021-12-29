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
        cv_image = bridge.imgmsg_to_cv2 (frame, 'mono8')
        cv_image = cv.rotate(cv_image, cv.ROTATE_180)
        # 밑라인
        img_list =[]
        img_list.append(np.var(cv_image[0:53, 450:480]))
        img_list.append(np.var(cv_image[53:106, 450:480]))   
        img_list.append(np.var(cv_image[106:159, 450:480]))
        img_list.append(np.var(cv_image[159:212, 450:480]))
        img_list.append(np.var(cv_image[212:265, 450:480]))  # center
        img_list.append(np.var(cv_image[265:318, 450:480]))
        img_list.append(np.var(cv_image[318:371, 450:480]))
        img_list.append(np.var(cv_image[371:424, 450:480]))
        img_list.append(np.var(cv_image[424:478, 450:480]))
        # 윗라인
        forward_list = []
        forward_list.append(np.var(cv_image[159:212, 420:450]))
        forward_list.append(np.var(cv_image[212:265, 420:450]))  # center
        forward_list.append(np.var(cv_image[265:318, 420:450]))

        # 분산이 크면 검정색과 흰색의 조합으로 봄
        tmp = max(img_list)
        index = img_list.index(tmp)
        print("밑 : ",img_list)
        print(index)
    
        tmp2 = max(forward_list)
        index2 = forward_list.index(tmp2)
        print("위 : ",forward_list)
        print(index2)
        # rospy.sleep(10)
        if tmp2 > 3000 :
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = (1-index2)/10

        elif tmp > 1600 :
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = (4-index)/10
    
        else :
            cmd_vel.linear.x = 0.01
            cmd_vel.angular.z = -0.2
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

