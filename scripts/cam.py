import cv2
import numpy as np
cap = cv2.VideoCapture(4)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FPS,30)
frameWidth = 720
frameHeight = 480
name = 'out'
def decode_fourcc(v):
    v = int(v)
    return "".join([chr((v >> 8*i) & 0xFF) for i in range(4)])
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
video = cv2.VideoWriter(name, fourcc, 30, (w,h))
while (1):
    ret,frame = cap.read()
    video.write(frame)
    cv2.imshow("frame",frame)
    k = cv2.waitKey(10)
    if k == ord('q'):
            break
cap.release()
video.release
cv2.destroyAllWindows()



