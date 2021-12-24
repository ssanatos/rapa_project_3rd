from flask import Flask, Response
import cv2 as cv

app = Flask(__name__)

# 스레드에서 -> 글로벌 변수로 넘겨 -> 디코딩에서 받아.

# 브라우저와 소통하기 위해 @app.route("/") 기입
@app.route("/")
def helloworld():
    str = "Hello World!"
    return str

# 외부 선언은 그냥 선언해도 되지만, 보기편하게 global로 선언했다.
global video_frame
video_frame = None
def encodeframe():
    # 계속적인 동작 구현
    # 이미지를 날릴 수 있게 인코딩으로 바꿔줌
    # imencode(확장자, 화면캡쳐 받은것, )
    global video_frame
    while True:
        ret, encoded_image = cv.imencode('.jpg', video_frame)
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encoded_image) + b'\r\n')
    return

@app.route("/streaming")
def streamframe():
    #멀티파트라는 형식으로 프래임으로 바운드를 해줌. 
    return Response(encodeframe(), mimetype = 'multipart/x-mixed-replace; boundary=frame')

# host가 ip. 다른데서도 막 접속하게 0.0.0.0 ---
# 저 플라스크를 서버인양 동작하기
# app.run(host = '0.0.0.0', port = '8000')
# 실행하면 터미널에 나오는 IP 클릭해서 들어가기.

def capture_frame():
    # /dev/video
    global video_frame
    cap = cv.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        # cv.imshow('webcam',frame)
        frame = cv.rotate(frame, cv.ROTATE_180 )
        video_frame = frame.copy()
        cv.waitKey(30)
        pass
    # return

import threading
if __name__ == '__main__':
    cap_thread = threading.Thread(target = capture_frame)
    cap_thread.daemon = True
    cap_thread.start()
    # cap_thread.join()
    app.run(host='0.0.0.0', port='8000')