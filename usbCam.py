#show the live of the usb cam

import cv2
import numpy as np

cap = cv2.VideoCapture(1)
#VGA (640x480): 90 fps
# cap.set(cv2.CAP_PROP_FORMAT, cv2.CV_8U)#8 bits per pixel

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# cap.set(cv2.CAP_PROP_FPS, 15)
# cap.set(cv2.CAP_PROP_MODE, 2)#0: 640x480, 1: 1280x720, 2: 1920x1080

# cap.set(cv2.CAP_PROP_FPS, 90)
# #gray scale
# cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
cap.set(cv2.CAP_PROP_CONTRAST, 50)
cap.set(cv2.CAP_PROP_SATURATION, 50)
# cap.set(cv2.CAP_PROP_BRIGHTNESS, -50)
# cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 100)
#-1;-10; -8 ;-2
cap.set(cv2.CAP_PROP_EXPOSURE, -4)
# cap.set(cv2.CAP_PROP_MONOCHROME, 1)
# cap.set(cv2.CAP_PROP_GAIN, 0)

#use CAP_PROP_SETTINGS to change the settings of the camera
# cap.set(cv2.CAP_PROP_MONOCHROME, 0)

# cap.set(cv2.CAP_PROP_FPS, 10)


gain = 0
import time
# cap.set(cv2.CAP_PROP_GAIN, -0.1)

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    print(frame.shape)
    
    # cap.set(cv2.CAP_PROP_EXPOSURE, gain)
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, expo)
    # cap.set(cv2.CAP_PROP_GAIN, gain)
    # cap.set(cv2.CAP_PROP_ROLL, CAP_PROP_ROLL)
    # cap.set(cv2.CAP_PROP_ISO_SPEED, CAP_PROP_ISO_SPEED)
    # cap.set(cv2.CAP_PROP_BRIGHTNESS, CAP_PROP_BRIGHTNESS)
    # gain += 1
    # print(gain)
    # time.sleep(0.1)
    #afficher les fps
    cv2.putText(frame, 'fps: '+str(int(cap.get(cv2.CAP_PROP_FPS))), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    # #afficher l'exposition
    # cv2.putText(frame, 'exposure: '+str(cap.get(cv2.CAP_PROP_EXPOSURE)), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    # #channel number
    # cv2.putText(frame, 'channels: '+str(cap.get(cv2.CAP_PROP_CHANNEL)), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    # #gain 
    # cv2.putText(frame, 'gain: '+str(cap.get(cv2.CAP_PROP_GAIN)), (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    
    # #mode
    # cv2.putText(frame, 'mode: '+str(cap.get(cv2.CAP_PROP_MODE)), (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()