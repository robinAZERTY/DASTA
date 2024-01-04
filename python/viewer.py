# # import matplotlib.pyplot as plt
# # import numpy as np
# # import quaternion
# # from mpl_toolkits.mplot3d import Axes3D
# # from matplotlib.animation import FuncAnimation
# # import json
# # import os
# # import JSONFileHandler




    
# # def main(telemetry_db):
    
# #     global h 
# #     h = JSONFileHandler.createJSONFileHandler(telemetry_db.name)

# #     position = np.array([0, 0, 0])
# #     velocity = np.array([0, 0, 0])
# #     orientation = quaternion.quaternion(1, 0, 0, 0)
# #     arrow_length = 0.3

# #     # Create the figure and 3D axis
# #     fig = plt.figure()
# #     ax = fig.add_subplot(111, projection='3d')

# #     # Set initial arrow directions
# #     arx = np.array([arrow_length, 0, 0]) @ quaternion.as_rotation_matrix(orientation)
# #     ary = np.array([0, arrow_length, 0]) @ quaternion.as_rotation_matrix(orientation)
# #     arz = np.array([0, 0, arrow_length]) @ quaternion.as_rotation_matrix(orientation)
# #     arv = velocity

# #     # Draw initial arrows
# #     ax.quiver(position[0], position[1], position[2], arx[0], arx[1], arx[2], color='r')
# #     ax.quiver(position[0], position[1], position[2], ary[0], ary[1], ary[2], color='g')
# #     ax.quiver(position[0], position[1], position[2], arz[0], arz[1], arz[2], color='b')
# #     ax.quiver(position[0], position[1], position[2], arv[0], arv[1], arv[2], color='c')
    
    
# #     def update(frame):          
# #         if h.new_data_available == True and h.last_data != []:
# #             h.new_data_available = False
# #             data = h.last_data.copy()
# #             #clear the dataBase
# #             telemetry_db.seek(0)
# #             json.dump([], telemetry_db, indent=4)
# #             telemetry_db.truncate()
# #             telemetry_db.flush()
# #             try:
# #                 data = data[-1]
# #             except:
# #                 pass
# #             # data = data[-1]
# #             # print(data)
            
# #             #get the last orientation
# #             orientation = quaternion.quaternion(data["orientation"][0], data["orientation"][1], data["orientation"][2], data["orientation"][3])
# #             #get the last position
# #             position = np.array([data["position"][0], data["position"][1], data["position"][2]])
# #             #get the last velocity
# #             velocity = np.array([data["velocity"][0], data["velocity"][1], data["velocity"][2]])
        
        
        
# #             orientation_rot = quaternion.as_rotation_matrix(orientation)

# #             # Update arrow directions
# #             arrow_length = 0.3
# #             arx = np.array([arrow_length, 0, 0]) @ orientation_rot
# #             ary = np.array([0, arrow_length, 0]) @ orientation_rot
# #             arz = np.array([0, 0, arrow_length]) @ orientation_rot
# #             arv = velocity

# #             # Clear previous arrows
# #             ax.clear()

# #             # Draw updated arrows
# #             ax.quiver(position[0], position[1], position[2], arx[0], arx[1], arx[2], color='r')
# #             ax.quiver(position[0], position[1], position[2], ary[0], ary[1], ary[2], color='g')
# #             ax.quiver(position[0], position[1], position[2], arz[0], arz[1], arz[2], color='b')
# #             ax.quiver(position[0], position[1], position[2], arv[0], arv[1], arv[2], color='c')

# #             # Set axis limits
# #             ax.set_xlim(-1, 1)
# #             ax.set_ylim(-1, 1)
# #             ax.set_zlim(-1, 1)

# #             # Set labels
# #             ax.set_xlabel('X')
# #             ax.set_ylabel('Y')
# #             ax.set_zlabel('Z')

# #             # Set title
# #             ax.set_title(f'Body position and orientation (Frame {frame})')

# #             # Auto-size the window
# #             fig.tight_layout()
        

# #     # Create the animation
# #     animation = FuncAnimation(fig, update, frames=range(360), interval=100, repeat=False)
# #     #auto size the window
# #     fig.tight_layout()
# #     #lim to + and - 1,5
# #     ax.set_xlim(-1.5, 1.5)
# #     ax.set_ylim(-1.5, 1.5)
# #     ax.set_zlim(-1.5, 1.5)
# #     # Show the plot
# #     plt.show()

# # if __name__ == "__main__":
    
# #     #make real time data base folder in read and write mode
# #     DBFolder = "real time data base"
# #     telemetryDBName = "telemetry.json"
    
# #     if not os.path.exists(DBFolder):
# #         os.makedirs(DBFolder)
    
# #     #create telemetry.json
# #     telemetry_db_path = os.path.join(DBFolder, telemetryDBName)
# #     telemetry_db = open(telemetry_db_path, "w+")
# #     main(telemetry_db)

# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from matplotlib import style
# import os
# import json
# import JSONFileHandler


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# arrowX = ax.quiver(0, 0, 0, 0, 0, 0, color='r')
# arrowY = ax.quiver(0, 0, 0, 0, 0, 0, color='g')


# DBFolder = "real time data base"
# telemetryDBName = "telemetry.json"



# #create telemetry.json
# telemetry_db_path = os.path.join(DBFolder, telemetryDBName)

# h = JSONFileHandler.createJSONFileHandler(telemetry_db_path)

import matplotlib.pyplot as plt 
import numpy as np 
import JSONFileHandler
import os
import json
import quaternion
  
x = np.linspace(0, 10*np.pi, 100) 
y = np.sin(x) 
  
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
telemetry_db_path = os.path.join(DBFolder, telemetryDBName)
h = JSONFileHandler.createJSONFileHandler(telemetry_db_path)
fig.canvas.draw() 
fig.canvas.flush_events() 

while True:
    if h.new_data_available:
        h.new_data_available=False
        if h.last_data == [] or h.last_data == None:
            continue
        data = h.last_data.copy()
        
        #clear the dataBase
        telemetry_db = open(telemetry_db_path, "w+")
        json.dump([], telemetry_db, indent=4)
        telemetry_db.truncate()
        telemetry_db.flush()
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