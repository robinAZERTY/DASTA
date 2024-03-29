import bluetoothTransmission
import time
import threading
import cv2


angularVel = [0.0, 0.0, 0.0]
lastAngularVel = [0.0, 0.0, 0.0]

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