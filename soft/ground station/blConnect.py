import socket
import time
import struct
import numpy as np
import cv2 as cv

s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

c = 0
while c < 10:
    try:
        s.connect(('C0:49:EF:CD:A7:D6', 1))
        print("Connected")
        break
    except:
        print("Can't connect")
        c += 1
        time.sleep(1)

if c == 10:
    print("Connection failed")
    exit()

END_LINE = "end_line\n".encode("utf-8")
DESCRIPTION_KEY="send_stream:".encode("utf-8")

#each message is ended by the string "end_line\n", so split the data by this bits

#lineContentType[0] = {"name", "type", "size}
lineContentType = []

TYPE_CHAR = 'c'
TYPE_INT = 'i'
TYPE_UNSIGNED_LONG_LONG = 'Q'
TYPE_FLOAT = 'f'
TYPE_DOUBLE = 'd'
TYPE_STRING = 's'
TYPE_VECTOR = 'v'
TYPE_MATRIX = 'm'

STD_TYPE_KEY = [TYPE_CHAR, TYPE_INT, TYPE_UNSIGNED_LONG_LONG, TYPE_FLOAT, TYPE_DOUBLE]


def decodeLineContentType(line):
    
    line = line.decode("utf-8")
    dataType = line.split(",")
    #remove empty string
    dataType = [dataType[i] for i in range(len(dataType)) if dataType[i] != '']
    #list of dict
    lineContentType = [{"name":None, "type":None, "size":None} for i in range(len(dataType))]
    for i in range(len(dataType)):
        dataType2 = dataType[i].split(":")
        lineContentType[i]["name"] = dataType2[0]
        lineContentType[i]["type"] = dataType2[1]
        lineContentType[i]["size"] = int(dataType2[2])
        if lineContentType[i]["type"] == TYPE_MATRIX:
            #add a "row" key to the dict
            lineContentType[i]["row"] = dataType2[3]
    return lineContentType

def myUnpack(data, dataFormat):
    
    size = dataFormat["size"]
    for type_key in STD_TYPE_KEY:
        if dataFormat["type"] == type_key:
            return struct.unpack(type_key, data[:size])[0]
        
        
    if dataFormat["type"] == TYPE_STRING:
        return data[::] # until the end of the data
    elif dataFormat["type"] == TYPE_VECTOR:
        length = int(size/4) #float
        oneElementSize = 4 #float
        v = np.zeros(length, dtype=np.float32)
        for j in range(length):
            v[j] = struct.unpack("f", data[j*oneElementSize:(j+1)*oneElementSize])[0]
        return v
    elif dataFormat["type"] == TYPE_MATRIX:
        height = int(dataFormat["row"])
        width = int(size/height/4) #float
        oneElementSize = 4 #float
        m=np.zeros((height,width), dtype=np.float32)
        for j in range(height):
            for k in range(width):
                a=struct.unpack("f", data[j*width*oneElementSize+k*oneElementSize:(j*width+k+1)*oneElementSize])[0]
                m[j][k] = a
        return m
    
    return None

def decodeLine(lineContentType, line):
    #the line is a chain of bytes, split it by the size of each data
    #lineContentType[0] = {"name", "type", "size}
    #when the data is a vector or a matrix, the size represent the dimension and the size of each element , for example, a vector of 3 float is "3*4" and a matrix of 3x3 float is "3*3*4"
    data = {}
    #add the name of the data in the dict, the first 32 bits are flags to indicate wich data is present
    stream_register = struct.unpack("I", line[:4])[0]
    line = line[4:]
    for i in range(len(lineContentType)):
        if stream_register & (1 << i):
            data[lineContentType[i]["name"]] = myUnpack(line[:lineContentType[i]["size"]], lineContentType[i])
            line = line[lineContentType[i]["size"]:]
    return data
    
    


#open a window
cv.namedWindow("EKF", cv.WINDOW_NORMAL)
cv.resizeWindow("EKF", 1000, 1000)
cv.moveWindow("EKF", 0, 0)

last_gps = np.array([0, 0])


#the data is received in a loop  
t0 = time.time()   

bytess = b""
while True:
    #read as string until the end of the line
    while END_LINE not in bytess:
        bytess += s.recv(1024)
        
    line = bytess.split(END_LINE)[0]
    bytess = bytess[len(line) + len(END_LINE):]
    
    if DESCRIPTION_KEY in line:
        print("Description received:")
        print(line.decode("utf-8"))
        lineContentType = decodeLineContentType(line[len(DESCRIPTION_KEY):])
        print(lineContentType)
    else:
        data = decodeLine(lineContentType, line)
        # print(data)
        #update the time
        if "gps" in data:
            last_gps = data["gps"]
        
        if data["t"] >= 99_900:
            break
        
        true_xy = np.array([data["true_X"][0], data["true_X"][1]])
        ekf_xy = np.array([data["ekf_X"][0], data["ekf_X"][1]])
        P_xy = data["ekf_P"][:2, :2]        
        #create a black image
        img = np.zeros((1000, 1000, 3), np.uint8)
        #add the time as text in the image
        ekf_t = data["t"]/1000
        cv.putText(img, str(ekf_t), (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        #add also the difference between the true time and the time of the message
        # cv.putText(img, str(time.time() - t0 - ekf_t), (10, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        #draw a circle
        cv.circle(img, (int(true_xy[0]*30+200), int(true_xy[1]*30+200)), 10, (0, 255, 0), 2)
        # cv.circle(img, (int(ekf_xy[0]*30+200), int(ekf_xy[1]*30+200)), 10, (0, 0, 255), 2)
        #draw a point where the ekf is
        cv.circle(img, (int(ekf_xy[0]*30+200), int(ekf_xy[1]*30+200)), 2, (0, 0, 255), 2)
        
        #draw the covariance as an ellipse
        w, v = np.linalg.eig(P_xy)
        w = np.sqrt(w)
        #draw the ellipse
        cv.ellipse(img, (int(ekf_xy[0]*30+200), int(ekf_xy[1]*30+200)), (int(w[0]*30), int(w[1]*30)), np.rad2deg(np.arctan2(v[1][0], v[0][0])), 0, 360, (0, 0, 255), 2)
        
        #draw a cross where the gps is
        cross_size = 10
        cv.line(img, (int(last_gps[0]*30+200-cross_size), int(last_gps[1]*30+200)), (int(last_gps[0]*30+200+cross_size), int(last_gps[1]*30+200)), (255, 0, 0), 2)
        cv.line(img, (int(last_gps[0]*30+200), int(last_gps[1]*30+200-cross_size)), (int(last_gps[0]*30+200), int(last_gps[1]*30+200+cross_size)), (255, 0, 0), 2)
        cv.imshow("EKF", img)
        
        #update the window
        cv.waitKey(1)

s.close()
input("Press enter to exit")