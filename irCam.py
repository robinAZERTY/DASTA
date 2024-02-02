import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
import time
#open WIN_20240117_13_11_07_Pro.mp4 and show its histogram in a window


cap = cv2.VideoCapture('WIN_20240117_13_11_07_Pro.mp4')


def getNewFrame():
    ret, frame = cap.read()
    return ret,np.uint8(frame)

def getExtrems(labels):
    endPoints = []
    skeletonLength = []
    distances = []
    centoids = []
    kernel = np.array([[1,1,1],[1,10,1],[1,1,1]])
    maxLabel = np.max(labels)
    
    for i in range(1,maxLabel+1):
        #mask the label
        mask = np.zeros(labels.shape)
        mask[labels==i] = 1
        distTransorm = cv2.distanceTransform(np.uint8(frame),cv2.DIST_L2,5)
        #find the centoid of the label
        M = cv2.moments(mask)
        if M["m00"] == 0:
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centoids.append((cY,cX))
        skeletonLength.append(np.sum(mask))
        #find the end points of the label
        endPointsLabel = np.uint16(np.argwhere(cv2.filter2D(np.uint8(mask),-1,kernel)==11))
        endPoints.append(endPointsLabel)
        #add the distance of the end points to the list
        distances.append(distTransorm[endPointsLabel[:,0],endPointsLabel[:,1]])
        
    return endPoints,skeletonLength,distances,centoids

    
# def findEndPoints(frame, maxLabelBeforNone=10):
#     frame2 = frame/np.max(frame)*255
#     frame2 = cv2.threshold(frame2,50,255,cv2.THRESH_BINARY)[1]
#     # print(np.average(frame2)  )
#     skemeton = skeletonize(frame2)
#     ret, labels = cv2.connectedComponents(np.uint8(skemeton))
#     if np.max(labels)>maxLabelBeforNone: #if there is too much labels, it's probably noise
#         return None,None
#     endPoints = getEndPoints(labels)

    # return endPoints,skemeton

ledNumber = 2

while(cap.isOpened()):
    ret,originalFrame = getNewFrame()
    if not ret:
        continue
    time.sleep(0.1)

    frame = cv2.cvtColor(originalFrame, cv2.COLOR_BGR2GRAY)
    frame = np.uint8(frame/np.max(frame)*255)
    frame = cv2.threshold(frame,75,255,cv2.THRESH_BINARY)[1]
    #find the differents components
    ret, labels = cv2.connectedComponents(np.uint8(frame))
    
    # if np.max(labels)>ledNumber:
    #     #dilater l'image
    #     kernel = np.ones((5,5),np.uint8)
    #     frame = cv2.dilate(frame,kernel,iterations = 1)
    
        
    skeleton = skeletonize(frame)
    ret, labels = cv2.connectedComponents(np.uint8(skeleton))
    
    if(np.max(labels)==ledNumber):
        extrem,skeletonLength,distances,centoids = getExtrems(labels)        
        #convert the frame to rgb
        frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)

        for segment_index in range(len(extrem)):
            if len(extrem[segment_index])==0:
                continue
            if (skeletonLength[segment_index]>4*np.max(distances[segment_index])):
                #draw the end points on the frame
                for point in extrem[segment_index]:
                    cv2.circle(originalFrame,(point[1],point[0]), 3, (0,0,255), -1)
            else:
                #draw the centoids on the frame       
                cv2.circle(originalFrame,(int(centoids[segment_index][1]),int(centoids[segment_index][0])), 3, (0,0,255), -1)
        
        
    
    
    
    
    #resize the frame
    originalFrame = cv2.resize(originalFrame,(originalFrame.shape[1]*2,originalFrame.shape[0]*2))
    cv2.imshow('frame',np.uint8(originalFrame))
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        