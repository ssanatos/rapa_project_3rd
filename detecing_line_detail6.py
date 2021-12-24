#! /usr/bin/env python3
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import numpy as np
import threading

# PC roscore , turtle bringup robot , turtle rosrun cv_camera cv_camera_node , chmod 777 *.py , python3 파일명.py

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

    # rospy.spin()  
    pass


def callback(frame):
    global cmd_vel

    if frame != None:
        cv_image = bridge.imgmsg_to_cv2 (frame, 'bgr8')
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

        collect = np.empty(shape=(3,19))
        for idxi, i in enumerate(range(354,448,32)):
            for idxk, k in enumerate(range(2,608,32)):
                row_gap = abs(np.mean(cv_image[k-4:k, i-4:i]) - np.mean(cv_image[k+28:k+32, i-4:i]))
                col_gap = abs(np.mean(cv_image[k-4:k, i-4:i]) - np.mean(cv_image[k-4:k, i+28:i+32]))
                collect[idxi,idxk] = (row_gap + col_gap)*((9.5-idxk)*(idxi+1)**2)
        cmd_vel.angular.z = sum(collect)*0.00018797
        cmd_vel.linear.x = 0.1
        pub.publish(cmd_vel)
    else:
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
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

