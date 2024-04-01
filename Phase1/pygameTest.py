import cv2

# open a window with sliders to control the angular velocity
#framesize of 300x300
cv2.namedWindow("CONTROL")
cv2.resizeWindow("CONTROL", 300, 800)

angularVel = [0.0, 0.0, 0.0]
rxmin = -1.0
rxmax = 1.0
rymin = -1.0
rymax = 1.0
rzmin = -1.0
rzmax = 1.0

#slider centered in the middle of the window
cv2.createTrackbar("X", "CONTROL", 0, 100, lambda x: angularVel.__setitem__(0, rxmin + (rxmax - rxmin) * x / 100))
cv2.createTrackbar("Y", "CONTROL", 0, 100, lambda x: angularVel.__setitem__(1, rymin + (rymax - rymin) * x / 100))
cv2.createTrackbar("Z", "CONTROL", 0, 100, lambda x: angularVel.__setitem__(2, rzmin + (rzmax - rzmin) * x / 100))
# but with default values to 50
cv2.setTrackbarPos("X", "CONTROL", 50)
cv2.setTrackbarPos("Y", "CONTROL", 50)
cv2.setTrackbarPos("Z", "CONTROL", 50)


#text fill in for pid parameters
#params = [kp,ki,kd,timeConstDerFilter,maxIntegral,min,max]
pidX = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
pidY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
pidZ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

minkp = 0.0
maxkp = 1.0
minki = 0.0
maxki = 1.0
minkd = 0.0
maxkd = 1.0
mintimeConstDerFilter = 0.0
maxtimeConstDerFilter = 1.0
maxmaxIntegral = 1.0
minmaxIntegral = 0.0
minmin = 0.0
maxmin = 1.0
minmax = 0.0
maxmax = 1.0


cv2.createTrackbar("kpX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(0, minkp + (maxkp - minkp) * x / 100))
cv2.createTrackbar("kiX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(1, minki + (maxki - minki) * x / 100))
cv2.createTrackbar("kdX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(2, minkd + (maxkd - minkd) * x / 100))
cv2.createTrackbar("timeConstDerFilterX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(3, mintimeConstDerFilter + (maxtimeConstDerFilter - mintimeConstDerFilter) * x / 100))
cv2.createTrackbar("maxIntegralX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(4, minmaxIntegral + (maxmaxIntegral - minmaxIntegral) * x / 100))
cv2.createTrackbar("minX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(5, minmin + (maxmin - minmin) * x / 100))
cv2.createTrackbar("maxX", "CONTROL", 0, 100, lambda x: pidX.__setitem__(6, minmax + (maxmax - minmax) * x / 100))

cv2.createTrackbar("kpY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(0, minkp + (maxkp - minkp) * x / 100))
cv2.createTrackbar("kiY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(1, minki + (maxki - minki) * x / 100))
cv2.createTrackbar("kdY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(2, minkd + (maxkd - minkd) * x / 100))
cv2.createTrackbar("timeConstDerFilterY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(3, mintimeConstDerFilter + (maxtimeConstDerFilter - mintimeConstDerFilter) * x / 100))
cv2.createTrackbar("maxIntegralY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(4, minmaxIntegral + (maxmaxIntegral - minmaxIntegral) * x / 100))
cv2.createTrackbar("minY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(5, minmin + (maxmin - minmin) * x / 100))
cv2.createTrackbar("maxY", "CONTROL", 0, 100, lambda x: pidY.__setitem__(6, minmax + (maxmax - minmax) * x / 100))

cv2.createTrackbar("kpZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(0, minkp + (maxkp - minkp) * x / 100))
cv2.createTrackbar("kiZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(1, minki + (maxki - minki) * x / 100))
cv2.createTrackbar("kdZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(2, minkd + (maxkd - minkd) * x / 100))
cv2.createTrackbar("timeConstDerFilterZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(3, mintimeConstDerFilter + (maxtimeConstDerFilter - mintimeConstDerFilter) * x / 100))
cv2.createTrackbar("maxIntegralZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(4, minmaxIntegral + (maxmaxIntegral - minmaxIntegral) * x / 100))
cv2.createTrackbar("minZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(5, minmin + (maxmin - minmin) * x / 100))
cv2.createTrackbar("maxZ", "CONTROL", 0, 100, lambda x: pidZ.__setitem__(6, minmax + (maxmax - minmax) * x / 100))



    
data_to_send = {}
while True:
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    if key == ord('s'):
        data_to_send["angular_velocity_command"] = angularVel
        data_to_send["pidX_params"] = pidX
        data_to_send["pidY_params"] = pidY
        data_to_send["pidZ_params"] = pidZ
        print(data_to_send)
