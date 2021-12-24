import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from std_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

def callback(data):
    global direction
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

    # 이상한 곳으로 가면 키보드로 통제
    key = cv.waitKey(1)
    if key == ord('w'):
        direction = 'GO'

    elif key == ord('s'):
        direction = 'STOP'

    elif key == ord('z'):
        direction = 'BACK'

    elif key == ord('a'):
        direction = 'LEFT'

    elif key == ord('d'):
        direction = 'RIGHT'

    if key == None:

        # 중앙 사각형 검정색이면
        if np.all(cv_image[320:330, 350:360]) < 70 or np.all(cv_image[320:330, 430:440]) < 70:
            direction = 'GO'
        # 왼쪽 위 사각형 검정이면
        elif np.all(cv_image[240:250, 350:360]) < 70:
            direction = 'LEFT'
        # 오른쪽 위 사각형 검정이면
        elif np.all(cv_image[400:410, 350:360]) < 70:
            direction = 'RIGHT'
        # 왼쪽 아래 사각형 검정색이면
        elif np.all(cv_image[80:90, 430:440]) < 70 or np.all(cv_image[160:170, 430:440]) < 70 or np.all(cv_image[240:250, 430:440]) < 70:
            direction = 'LEFT'
        # 오른쪽 아래사각형 검정색
        elif np.all(cv_image[400:410, 430:440]) < 70 or np.all(cv_image[480:490, 430:440]) < 70 or np.all(cv_image[560:570, 430:440]) < 70:
            direction = 'RIGHT'
        # 중앙 아래 사각형이 밖으로 나가면 정지
        elif np.all(cv_image[320:330, 430:440]) > 70 and np.all(cv_image[320:330, 430:440]) < 120 :
            direction = 'STOP'
        else:
            pass
           
    # pub = rospy.Publisher('/motor_commands', String, queue_size=10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    pub.publish(direction)

def main():
    rospy.init_node('planer_node')
    rospy.Subscriber('/cv_camera/image_raw',Image,callback)
    rospy.spin()
    pass

if __name__ == "__main__" :
    direction = 'GO'
    main()       
    pass


