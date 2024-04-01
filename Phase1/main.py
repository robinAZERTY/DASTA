import bluetoothTransmission
import time
import threading
import cv2
import numpy as np


class Plotter:
	def __init__(self, plot_width, plot_height, num_plot_values):
	    self.width = plot_width
	    self.height = plot_height
	    self.color_list = [(255, 0 ,0), (0, 250 ,0),(0, 0 ,250),
	    			(0, 255 ,250),(250, 0 ,250),(250, 250 ,0),
	    			(200, 100 ,200),(100, 200 ,200),(200, 200 ,100)]
	    self.color  = []
	    self.val = []
	    self.plot = np.ones((self.height, self.width, 3))*255

	    for i in range(num_plot_values):
	    	self.color.append(self.color_list[i])

	# Update new values in plot
	def multiplot(self, val, label = "plot"):
		self.val.append(val)
		while len(self.val) > self.width:
			self.val.pop(0)

		self.show_plot(label)

    # Show plot using opencv imshow
	def show_plot(self, label):
		self.plot = np.ones((self.height, self.width, 3))*255
		cv2.line(self.plot, (0, int(self.height/2) ), (self.width, int(self.height/2)), (0,255,0), 1)
		for i in range(len(self.val)-1):
			for j in range(len(self.val[0])):
				cv2.line(self.plot, (i, int(self.height/2) - self.val[i][j]), (i+1, int(self.height/2) - self.val[i+1][j]), self.color[j], 1)

		cv2.imshow(label, self.plot)
		cv2.waitKey(10)


angularVel = [0.0, 0.0, 0.0]
lastAngularVel = [0.0, 0.0, 0.0]

pX = Plotter(400, 200, 2)
pY = Plotter(400, 200, 2)
pZ = Plotter(400, 200, 2)
pMotor = Plotter(400, 200, 4)

def main():
    
    global angularVel
    global lastAngularVel
    while not bluetoothTransmission.inited:
        time.sleep(0.1)
    
    time.sleep(1)
    bluetoothTransmission.data_to_send.append({"user_event": 7, "send_stream_delay": 100}) #ENABLE_SENSOR_STREAM
    time.sleep(3)
    bluetoothTransmission.data_to_send.append({"user_event": 3}) #START_STREAM

    cv2.namedWindow("frame")
    while(True):
        if len(bluetoothTransmission.received_data) > 0:
            for data in bluetoothTransmission.received_data:
                if "gyro" in data and "angular_vel_command" in data:
                    pX.multiplot([data["gyro"][0], data["angular_vel_command"][0]], "X")
                    pY.multiplot([data["gyro"][1], data["angular_vel_command"][1]], "Y")
                    pZ.multiplot([data["gyro"][2], data["angular_vel_command"][2]], "Z")
                if "m1PWM" in data and "m2PWM" in data and "m3PWM" in data and "m4PWM" in data:
                    pMotor.multiplot([data["m1PWM"], data["m2PWM"], data["m3PWM"], data["m4PWM"]], "Motors")
                    
            bluetoothTransmission.received_data = []
        
        increment = 0.01
        key = cv2.waitKey(1)
        if key == ord('z'):
            angularVel[0] += increment
        elif key == ord('Z'):
            angularVel[0] -= increment
        elif key == ord('q'):
            angularVel[1] += increment
        elif key == ord('Q'):
            angularVel[1] -= increment
        elif key == ord('d'):
            angularVel[2] += increment
        elif key == ord('D'):
            angularVel[2] -= increment

        
        if angularVel != lastAngularVel:
            bluetoothTransmission.data_to_send.append({"angular_velocity_command": angularVel})
            lastAngularVel = angularVel.copy()
        
        
if __name__ == "__main__":
    bluetoothTransmission.main()#init the bluetooth
    threading.Thread(target=main).start()