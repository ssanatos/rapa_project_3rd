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
        rospy.init_node('img_cv_node')
        # 이 정보를 받아서, 이 변수로 , 이 함수에 보낼것이다.
        rospy.Subscriber('/cv_camera/image_raw',Image,callback)
        pass
    else :
        # STOP
        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)

    # rospy.spin()  
    pass



def callback(frame):
    global cmd_vel
    # cmd_vel.linear.x = 0.1
    # pub.publish(cmd_vel)

    # 라이다를 토픽으로 받는다면, 카메라 또한 토픽으로 받겠지.
    # bringup rpicamera.launch 하려했더니 kinetic만 지원된다는 에러.
    # 터미널에서 rosrun cv_camera cv_camera_node 를 실행해야 카메라가 돈다.
    # video_capture = cv.VideoCapture(0)
    # if video_capture.isOpened():

    # # while True and video_capture.isOpened():
    #     ret, frame = video_capture.read()
    if frame != None:

        # cv_image = bridge.imgmsg_to_cv2 (frame, cv.IMREAD_GRAYSCALE) # 이 코드는 에러남
        cv_image = bridge.imgmsg_to_cv2 (frame, 'bgr8')
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        # cv.imshow('now driving', cv_image)

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
        # cmd_vel.linear.x = 0.1

        # pub = rospy.Publisher('/motor_commands', String, queue_size=10)
        pub.publish(cmd_vel)
    else:
        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)
        print('카메라 연결 안됐음')
    return


if __name__ == '__main__':
    rospy.init_node('img_cv_node')
    cmd_vel = Twist()
    sub = rospy.Subscriber('/scan', LaserScan, scan)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()


