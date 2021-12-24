import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()
# direction = None
def callback(data):
    # global direction
    cv_image = bridge.imgmsg_to_cv2 (data, 'bgr8')
    cv.imshow('now driving', cv_image)
        
    rect_img = cv.rectangle(cv_image, (0,790), (150,799), (0,0,255), 3)
    rect_img = cv.rectangle(cv_image, (650,790), (799,799), (0,0,255), 3)
    # cv.imshow('now driving',rect_img)

    left_img = cv_image[0:150, 790:799]
    # print(left_img)
    right_img = cv_image[650:799, 790:799]
    # print(left_img)
    
    # if np.all(left_img) > 190 :
    # if sum(left_img) > 190*3 :
    if np.any(left_img) > 190 :
        direction = 'LEFT'
        pass
    elif np.any(right_img) > 190:
        direction = 'RIGHT'
        pass
    else :
        direction = 'GO'
        pass
           
    pub = rospy.Publisher('/motor_commands', String, queue_size=10)
    pub.publish(direction)

def main():
    rospy.init_node('planer_node')
    # 이 정보를 받아서, 이 변수로 , 이 함수에 보낼것이다.
    rospy.Subscriber('/camera/image_raw',Image,callback)
    rospy.spin()
    pass

if __name__ == "__main__" :
    direction = None
    main()       
    pass




