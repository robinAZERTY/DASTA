import bluetoothTransmission
import json
import os
import matplotlib.pyplot as plt
import numpy as np
import quaternion
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import json
import os
import time
import JSONFileHandler

def mainBis(telemetry_db):
        
    plt.ion() 
    fig = plt.figure() 
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_zlim(-1.5, 1.5)
    fig.tight_layout()

    #create 3d line to show the orientation
    xdirLine, = ax.plot([0, 0], [0, 0], [0, 0], color='r')
    ydirLine, = ax.plot([0, 0], [0, 0], [0, 0], color='g')
    zdirLine, = ax.plot([0, 0], [0, 0], [0, 0], color='b')


    DBFolder = "real time data base"
    telemetryDBName = "telemetry.json"
    # telemetry_db_path = os.path.join(DBFolder, telemetryDBName)
    # h = JSONFileHandler.createJSONFileHandler(telemetry_db_path)
    fig.canvas.draw() 
    fig.canvas.flush_events() 

    while True:
        time.sleep(0.1)
        #get the last data
        data = open(telemetry_db.name, "r")
        data = json.load(data) 
        # print(data)
        if data == [] or data == None:
            continue
        try:
            data = data[-1]
        except:
            pass
        
        #get the last orientation
        orientation = quaternion.quaternion(data["orientation"][0], data["orientation"][1], data["orientation"][2], data["orientation"][3])
        #get the last position
        position = np.array([data["position"][0], data["position"][1], data["position"][2]])
        #get the last velocity
        velocity = np.array([data["velocity"][0], data["velocity"][1], data["velocity"][2]])
    
    
    
        orientation_rot = quaternion.as_rotation_matrix(orientation)

        # Update arrow directions
        arrow_length = 0.3
        arx = np.array([arrow_length, 0, 0]) @ orientation_rot
        ary = np.array([0, arrow_length, 0]) @ orientation_rot
        arz = np.array([0, 0, arrow_length]) @ orientation_rot
        
        #update the 3d line        
        xdirLine.set_data([position[0], position[0]+arx[0]], [position[1], position[1]+arx[1]])
        ydirLine.set_data([position[0], position[0]+ary[0]], [position[1], position[1]+ary[1]])
        zdirLine.set_data([position[0], position[0]+arz[0]], [position[1], position[1]+arz[1]])
        
        xdirLine.set_3d_properties([position[2], position[2]+arx[2]])
        ydirLine.set_3d_properties([position[2], position[2]+ary[2]])
        zdirLine.set_3d_properties([position[2], position[2]+arz[2]])
        

            
        fig.canvas.draw() 
        fig.canvas.flush_events() 
    

if __name__ == "__main__":
    #make real time data base folder in read and write mode
    DBFolder = "real time data base"
    commandDBName = "userCommand.json"
    telemetryDBName = "telemetry.json"
    
    if not os.path.exists(DBFolder):
        os.makedirs(DBFolder)
    
    #create the dataBase files
    #reate the userCommand.json file
    userCommand_db_path = os.path.join(DBFolder, commandDBName)
    userCommand_db = open(userCommand_db_path, "w+")
    json.dump([], userCommand_db, indent=4)
    userCommand_db.flush()
    userCommand_db.seek(0)
    
    #create telemetry.json
    telemetry_db_path = os.path.join(DBFolder, telemetryDBName)
    telemetry_db = open(telemetry_db_path, "w+")
    #remove all content
    telemetry_db.truncate()
    telemetry_db.flush()
    telemetry_db.seek(0)
    # bluetoothTransmission.main(telemetry_db, userCommand_db)
    # mainBis(telemetry_db)
    h2 = JSONFileHandler.createJSONFileHandler(userCommand_db.name)
    
        
    connection = bluetoothTransmission.connect('FC:F5:C4:27:09:16')
    if connection == None:
        exit()

    bluetoothTransmission.threading.Thread(target=bluetoothTransmission.receiveTask, args=(connection, telemetry_db)).start()
    bluetoothTransmission.threading.Thread(target=bluetoothTransmission.sendTask, args=(connection,userCommand_db,h2)).start()
    bluetoothTransmission.threading.Thread(target=mainBis, args=(telemetry_db,)).start()
    