import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

def callback(data):
    global direction
    cv_image = bridge.imgmsg_to_cv2 (data, cv.IMREAD_GRAYSCALE)
    cv.imshow('now driving', cv_image)
        
    rect_img = cv.rectangle(cv_image, (260, 430), (270, 440), (0,0,255), 3)
    rect_img = cv.rectangle(cv_image, (370, 430), (380, 440), (0,0,255), 3)
    rect_img = cv.rectangle(cv_image, (210, 430), (220, 440), (0,0,255), 3)
    rect_img = cv.rectangle(cv_image, (420, 430), (430, 440), (0,0,255), 3)
    cv.imshow('now driving',rect_img)

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

        left_img = cv_image[260:270, 430:440]
        right_img = cv_image[210:220, 430:440]
        left_img2 = cv_image[260:270, 430:440]
        right_img2 = cv_image[420:430, 430:440]

        left_mean = left_img.mean()
        right_mean = right_img.mean()
        left_mean2 = left_img2.mean()
        right_mean2 = right_img2.mean()
        
        if left_mean < 60 or left_mean2 < 60 :
            direction = 'LEFT'
            pass
        elif right_mean < 60 or right_mean2 < 60 :
            direction = 'RIGHT'
            pass
        else :
            direction = 'GO'
            pass
           
    pub = rospy.Publisher('/motor_commands', String, queue_size=10)
    pub.publish(direction)

def main():
    rospy.init_node('planer_node')
    rospy.Subscriber('/cv_camera/image_raw',Image,callback)
    rospy.spin()
    pass

if __name__ == "__main__" :
    direction = None
    main()       
    pass


