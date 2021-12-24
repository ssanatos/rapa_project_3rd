#! /usr/bin/env python3
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import numpy as np
import threading
from flask import Flask, Response

app = Flask(__name__)

global video_frame
video_frame = None
def encodeframe():
    global video_frame
    while True:
        ret, encoded_image = cv.imencode('.jpg', video_frame)
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encoded_image) + b'\r\n')
    return

@app.route("/")
def streamframe():
    return Response(encodeframe(), mimetype = 'multipart/x-mixed-replace; boundary=frame')


def capture_frame():
    global video_frame
    cap = cv.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv.rotate(frame, cv.ROTATE_180 )
        video_frame = frame.copy()
        cv.waitKey(30)
        pass


bridge = CvBridge()
def scan(data):
    global cmd_vel

    laser_range = data.ranges[0:10]
    laser_arr = np.array(laser_range)
    no_hindrance = np.count_nonzero(laser_arr)
    result = np.count_nonzero(laser_arr >= 0.2 )

    if result > 0 or no_hindrance == 10 :
        # MOVE
        cmd_vel.linear.x = 0.1
        pass
    else :
        # STOP
        cmd_vel.linear.x = 0.0
    pub.publish(cmd_vel)

    # rospy.spin()  
    pass


def callback(frame):
    global cmd_vel

    if frame != None:
        cv_image = bridge.imgmsg_to_cv2 (frame, 'bgr8')
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        cv_image = cv.rotate(cv_image, cv.ROTATE_180 )

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
    cap_thread = threading.Thread(target = capture_frame)
    rospy.init_node('img_cv_node')
    cmd_vel = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    first = threading.Thread(target=subscrb, args=('/scan',LaserScan, scan))
    second = threading.Thread(target=subscrb2, args=('/cv_camera/image_raw',Image, callback))
    cap_thread.daemon = True
    first.daemon = True
    second.daemon = True
    cap_thread.start()
    app.run(host='0.0.0.0', port='8000')
    first.start()
    second.start()
    first.join()
    second.join()
    # rospy.spin()
