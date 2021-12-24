#! /usr/bin/env python3

import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

def callback(data):
    left_wheel_pub = rospy.Publisher('/left_wheel_controller/command', Float64, queue_size=10)
    right_wheel_pub = rospy.Publisher('/right_wheel_controller/command', Float64, queue_size=10)

    cv_image = bridge.imgmsg_to_cv2 (data, cv.IMREAD_GRAYSCALE)
    cv.imshow('now driving', cv_image)
    # 윗열    
    rect_img = cv.rectangle(cv_image, (240, 350), (250, 360), (0,0,255), 3) #왼쪽
    rect_img = cv.rectangle(cv_image, (320, 350), (330, 360), (0,0,255), 3) #중앙
    rect_img = cv.rectangle(cv_image, (400, 350), (410, 360), (0,0,255), 3) #오른쪽
    
    # 아랫열
    rect_img = cv.rectangle(cv_image, (80, 430), (90, 440), (0,0,255), 3)   #왼쪽
    rect_img = cv.rectangle(cv_image, (160, 430), (170, 440), (0,0,255), 3) #왼쪽
    rect_img = cv.rectangle(cv_image, (240, 430), (250, 440), (0,0,255), 3) #왼쪽
    rect_img = cv.rectangle(cv_image, (320, 430), (330, 440), (0,0,255), 3) #중앙
    rect_img = cv.rectangle(cv_image, (400, 430), (410, 440), (0,0,255), 3) #오른쪽
    rect_img = cv.rectangle(cv_image, (480, 430), (490, 440), (0,0,255), 3) #오른쪽
    rect_img = cv.rectangle(cv_image, (560, 430), (570, 440), (0,0,255), 3) #오른쪽
    cv.imshow('now driving',rect_img)


    # 중앙 사각형 검정색이면 전진
    if np.all(cv_image[320:330, 350:360]) < 70 or np.all(cv_image[320:330, 430:440]) < 70:
        left_wheel_pub.publish(1)
        right_wheel_pub.publish(1)

    # 왼쪽 위 사각형 검정이면 좌로
    elif np.all(cv_image[240:250, 350:360]) < 70:
        left_wheel_pub.publish(-1)
        right_wheel_pub.publish(1)      

    # 오른쪽 위 사각형 검정이면 우로
    elif np.all(cv_image[400:410, 350:360]) < 70:
        left_wheel_pub.publish(1)
        right_wheel_pub.publish(-1)

    # 왼쪽 아래 사각형 검정색이면 좌로
    elif np.all(cv_image[80:90, 430:440]) < 70 or np.all(cv_image[160:170, 430:440]) < 70 or np.all(cv_image[240:250, 430:440]) < 70:
        left_wheel_pub.publish(-1)
        right_wheel_pub.publish(1)      

    # 오른쪽 아래사각형 검정색 우로
    elif np.all(cv_image[400:410, 430:440]) < 70 or np.all(cv_image[480:490, 430:440]) < 70 or np.all(cv_image[560:570, 430:440]) < 70:
        left_wheel_pub.publish(1)
        right_wheel_pub.publish(-1)

    # 중앙 아래 사각형이 밖으로 나가면 정지
    elif np.all(cv_image[320:330, 430:440]) > 70 and np.all(cv_image[320:330, 430:440]) < 120 :
        left_wheel_pub.publish(0.0)
        right_wheel_pub.publish(0.0)
    # else:
    #     pass
    rospy.sleep(0.01)



def main():
    rospy.init_node('img_cv_node')
    rospy.Subscriber('/cv_camera/image_raw',Image,callback)
    rospy.spin()  # 로봇 안에서 돌릴 때는 이게 함수를 다시 불러냄.
    pass



if __name__ == "__main__" :
    main()       
    pass





===================================================


#! /usr/bin/env python3
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import numpy as np

bridge = CvBridge()

def callback(data):

    # 라이다 코드 함수로 넣기.

    # laser_range = data.ranges[0:10]
    # laser_arr = np.array(laser_range)
    # result = np.count_nonzero(laser_arr >= 0.2 )

    # if result > 0:
    #     # MOVE
    #     pass
    # else :
    #     # STOP
    #     cmd_vel.linear.x = 0.0

    cv_image = bridge.imgmsg_to_cv2 (data, cv.IMREAD_GRAYSCALE)
    cv.imshow('now driving', cv_image)
    cmd_vel = Twist()
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

            
    # pub = rospy.Publisher('/motor_commands', String, queue_size=10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    pub.publish(cmd_vel)

def main():
    rospy.init_node('img_cv_node')
    rospy.Subscriber('/cv_camera/image_raw',Image,callback)
    rospy.spin()
    pass

if __name__ == "__main__" :
    main()       
    pass


==================================================================================================================



#! /usr/bin/env python3
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import numpy as np

bridge = CvBridge()

def callback(data):
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.1
    pub.publish(cmd_vel)

    # 라이다 코드 함수로 넣기.
    laser_range = data.ranges[0:10]
    laser_arr = np.array(laser_range)
    result = np.count_nonzero(laser_arr >= 0.2 )

    if result > 0:
        # MOVE
        pass
    else :
        # STOP
        cmd_vel.linear.x = 0.0

  
    video_capture = cv.VideoCapture(0)
    if video_capture.isOpened():

    # while True and video_capture.isOpened():
        ret, frame = video_capture.read()        # ret 값은 None이 아닌데, frame은 None이다.
        if frame != None:               
    
            # cv_image = bridge.imgmsg_to_cv2 (frame, cv.IMREAD_GRAYSCALE)
            ros_image = bridge.imgmsg_to_cv2 (frame, 'bgr8')
            cv_image = cv.cvtColor(ros_image, cv.COLOR_BGR2GRAY)
            cv.imshow('now driving', cv_image)

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

            # pub = rospy.Publisher('/motor_commands', String, queue_size=10)
            pub.publish(cmd_vel)
    else:
        print('연결 안됐음')

if __name__ == '__main__':
    rospy.init_node('img_cv_node')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()

===========================================================================================
#! /usr/bin/env python3
# move_controller
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

bridge = CvBridge()
# direction = None
def callback(data):
    # global direction
    cv_image = bridge.imgmsg_to_cv2 (data, 'bgr8')
    cv.imshow('callback', cv_image)
    key = cv.waitKey(5)
    left_wheel_pub = rospy.Publisher('/left_wheel_controller/command', Twist, queue_size=10)
    right_wheel_pub = rospy.Publisher('/right_wheel_controller/command', Twist, queue_size=10)
    if key == ord('w'):
        left_wheel_pub.publish(1)
        right_wheel_pub.publish(1)
        # pass     # GO
    else :
        pass
    # rospy.sleep(0.3)

def main():
    rospy.init_node('planer_node')
    # 이 정보를 받아서, 이 변수로 , 이 함수에 보낼것이다.
    rospy.Subscriber('/cv_camera/image_raw',Image,callback)
    rospy.spin()
    pass

if __name__ == "__main__" :
    # direction = None # 글로벌로 디렉션 선언하면 키 한번만 누르면 명령계속먹고, 디렉션을 함수안에서 초기화하면 방향키를 계속 누르는 방식.
    main()       
    pass

