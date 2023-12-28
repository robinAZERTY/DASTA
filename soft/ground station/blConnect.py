import socket
import time
import struct
import numpy as np
import matplotlib.pyplot as plt

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
DESCRIPTION_KEY="description:".encode("utf-8")

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


def decodeLineContentType(line):
    
    line = line.decode("utf-8")
    dataType = line.split(";")
    #remove empty string
    dataType = [dataType[i] for i in range(len(dataType)) if dataType[i] != '']
    #list of dict
    lineContentType = [{"name":None, "type":None, "size":None} for i in range(len(dataType))]
    for i in range(len(dataType)):
        lineContentType[i]["name"] = dataType[i][0]
        lineContentType[i]["type"] = dataType[i][1]
        lineContentType[i]["size"] = dataType[i][2:]
    return lineContentType

def decodeLine(lineContentType, line):
    #the line is a chain of bytes, split it by the size of each data
    #lineContentType[0] = {"name", "type", "size}
    #when the data is a vector or a matrix, the size represent the dimension and the size of each element , for example, a vector of 3 float is "3*4" and a matrix of 3x3 float is "3*3*4"
    data = {}
    #add the name of the data in the dict
    for i in range(len(lineContentType)):
        data[lineContentType[i]["name"]] = None
        
    
    for i in range(len(lineContentType)):
        data[lineContentType[i]["name"]] = None
        if lineContentType[i]["type"] == TYPE_VECTOR or lineContentType[i]["type"] == TYPE_MATRIX:
            splitSize = lineContentType[i]["size"].split("*")
            size = 1
            for j in range(len(splitSize)):
                size *= int(splitSize[j])
        else:
            size = int(lineContentType[i]["size"])
            
        if lineContentType[i]["type"] == TYPE_CHAR:
            data[lineContentType[i]["name"]] = struct.unpack("c", line[:size])
        elif lineContentType[i]["type"] == TYPE_INT:
            data[lineContentType[i]["name"]] = struct.unpack("i", line[:size])
        elif lineContentType[i]["type"] == TYPE_UNSIGNED_LONG_LONG:
            data[lineContentType[i]["name"]] = struct.unpack("Q", line[:size])[0]
        elif lineContentType[i]["type"] == TYPE_FLOAT:
            data[lineContentType[i]["name"]] = struct.unpack("f", line[:size])
        elif lineContentType[i]["type"] == TYPE_DOUBLE:
            data[lineContentType[i]["name"]] = struct.unpack("d", line[:size])
        elif lineContentType[i]["type"] == TYPE_STRING:
            data[lineContentType[i]["name"]] = line[::] # until the end of the line
        elif lineContentType[i]["type"] == TYPE_VECTOR:
            length = int(lineContentType[i]["size"].split("*")[0])
            oneElementSize = int(lineContentType[i]["size"].split("*")[1])
            data[lineContentType[i]["name"]] = [None for j in range(length)]
            for j in range(length):
                data[lineContentType[i]["name"]][j] = struct.unpack("f", line[j*oneElementSize:(j+1)*oneElementSize])[0]
        elif lineContentType[i]["type"] == TYPE_MATRIX:
            height = int(lineContentType[i]["size"].split("*")[0])
            width = int(lineContentType[i]["size"].split("*")[1])
            oneElementSize = int(lineContentType[i]["size"].split("*")[2])
            data[lineContentType[i]["name"]] = np.array((height,width))
            for j in range(height):
                for k in range(width):
                    data[lineContentType[i]["name"]][j][k] = struct.unpack("f", line[j*width*oneElementSize+k*oneElementSize:(j*width+k+1)*oneElementSize])
        
        line = line[size:]
        
    return data
    
#the X vector contains the position x, y and the orientation theta        
#begin an animation with the position of the robot, display also the time
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
x = []
y = []
theta = []
pline, = ax.plot(x, y, 'r-')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Robot position')
plt.axis([-10, 10, -10, 10])
plt.show()

#the data is received in a loop     

bytess = b""
while True:
    #read as string until the end of the line
    while END_LINE not in bytess:
        bytess += s.recv(1024)
        
    line = bytess.split(END_LINE)[0]
    bytess = bytess[len(line) + len(END_LINE):]
    
    if DESCRIPTION_KEY in line:
        print("Description received:")
        lineContentType = decodeLineContentType(line[len(DESCRIPTION_KEY):])
        print(lineContentType)
    else:
        # print("Data received:")
        # print(line)
        data = decodeLine(lineContentType, line)
        #update the time
        fig.suptitle("Time: " + str(data["t"]))
        x.append(data["X"][0])
        y.append(data["X"][1])
        theta.append(data["X"][2])
        
        if data["t"] >= 99_900:
            break
        
pline.set_xdata(x)
pline.set_ydata(y)
fig.canvas.draw()
fig.canvas.flush_events()
s.close()
input("Press enter to exit")
        