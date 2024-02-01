import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
#open WIN_20240117_13_11_07_Pro.mp4 and show its histogram in a window


cap = cv2.VideoCapture('WIN_20240117_13_11_07_Pro.mp4')


def getNewFrame():
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return ret,np.uint8(frame)

def getEndPoints(labels):
    endPoints = []
    kernel = np.array([[1,1,1],[1,10,1],[1,1,1]])
    maxLabel = np.max(labels)
    
    for i in range(1,maxLabel+1):
        #mask the label
        mask = np.zeros(labels.shape)
        mask[labels==i] = 1
        #find the end points of the label
        endPointsLabel = np.uint16(np.argwhere(cv2.filter2D(np.uint8(mask),-1,kernel)==11))
        endPoints.append(endPointsLabel)
    return endPoints 

    
def findEndPoints(frame, maxLabelBeforNone=10):
    frame2 = frame/np.max(frame)*255
    frame2 = cv2.threshold(frame2,50,255,cv2.THRESH_BINARY)[1]
    # print(np.average(frame2)  )
    skemeton = skeletonize(frame2)
    ret, labels = cv2.connectedComponents(np.uint8(skemeton))
    if np.max(labels)>maxLabelBeforNone: #if there is too much labels, it's probably noise
        return None,None
    endPoints = getEndPoints(labels)

    return endPoints,skemeton


while(cap.isOpened()):
    ret,frame = getNewFrame()
    if not ret:
        continue
    # time.sleep(0.5)
    
    newEndPoints,skeleton = findEndPoints(frame)
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    if newEndPoints is not None:
        for line in newEndPoints:
            for point in line:
                cv2.circle(frame,(point[1],point[0]),3,(0,0,255),-1)
            
    if skeleton is not None:
        #add the skeleton to the frame
        frame[skeleton==1] = (255,0,0)
            
    #resize the frame
    frame = cv2.resize(frame,(frame.shape[1]*2,frame.shape[0]*2))
    cv2.imshow('frame',frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        