import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

bridge = CvBridge()
# direction = None
def callback(data):
    # global direction
    cv_image = bridge.imgmsg_to_cv2 (data, 'bgr8')
    cv.imshow('callback', cv_image)
    key = cv.waitKey(5)
    direction = None
    if key == ord('w'):
        direction = 'GO'
        # pass     # GO
    elif key == ord('s'):
        direction = 'STOP'
        # pass     # STOP
    elif key == ord('z'):
        direction = 'BACK'
        # pass     # BACK
    elif key == ord('a'):
        direction = 'LEFT'
        # pass     # LEFT
    elif key == ord('d'):
        direction = 'RIGHT'
        # pass     # RIGHT
    else :
        pass
    pub = rospy.Publisher('/motor_commands', String, queue_size=1)
    pub.publish(direction)
    # rospy.sleep(0.3)

def main():
    rospy.init_node('planer_node')
    # 이 정보를 받아서, 이 변수로 , 이 함수에 보낼것이다.
    rospy.Subscriber('/camera/image_raw',Image,callback)
    rospy.spin()
    pass

if __name__ == "__main__" :
    # direction = None # 글로벌로 디렉션 선언하면 키 한번만 누르면 명령계속먹고, 디렉션을 함수안에서 초기화하면 방향키를 계속 누르는 방식.
    main()       
    pass