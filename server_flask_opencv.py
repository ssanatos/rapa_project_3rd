from flask import Flask, Response

app = Flask(__name__)

@app.route("/")
def helloworld():
    str = "Hello World! seohyeonho"
    return str

global video_frame
video_frame = None
import cv2 as cv
def encodeframe():
    global video_frame
    while True:
        ret, encoded_image = cv.imencode('.jpg', video_frame)
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
                bytearray(encoded_image) + b'\r\n')
    return 

@app.route('/streaming')
def streamframe():
    return Response(encodeframe(), mimetype='multipart/x-mixed-replace; boundary=frame')

def captureframe():
    # /dev/video0
    global video_frame
    cap = cv.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        # cv.imshow('webcam', frame)
        frame = cv.rotate(frame, cv.ROTATE_180 )
        video_frame = frame.copy()
        cv.waitKey(30)
        pass
    # return 

import threading
if __name__ == '__main__':
    cap_thread = threading.Thread(target=captureframe)
    cap_thread.daemon = True
    cap_thread.start()
    # cap_thread.join()
    app.run(host='0.0.0.0', port='8000')
    pass

