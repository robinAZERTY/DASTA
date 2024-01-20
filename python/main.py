import bluetoothTransmission
import calibration
import threading
import time


def linkBL_and_Calibration():
    #waite until the bluetooth is connected
    while not bluetoothTransmission.inited:
        time.sleep(0.5)
    #begin the sensor stream at 100Hz
    bluetoothTransmission.data_to_send.append({"user_event": 7, "send_stream_delay": 10})
    time.sleep(1)
    bluetoothTransmission.data_to_send.append({"user_event": 3})
    while True:
        #check bluetoothTransmission.received_data 
        print("len(bluetoothTransmission.received_data): ", len(bluetoothTransmission.received_data))
        if len(bluetoothTransmission.received_data) > 0:
            if len(bluetoothTransmission.received_data[0])>0:
                received = bluetoothTransmission.received_data[0][0]
                #check is gyro and accel are in received
                if "gyro" in received and "acc" in received and "time" in received:
                    print("received gyro and accel")
                    calibration.time = received["time"]
                    calibration.gyr = received["gyro"]
                    calibration.acc = received["acc"]
                    calibration.new_proprio = True
            
            if len(bluetoothTransmission.received_data)>0:
                if len(bluetoothTransmission.received_data[0])>0:
                    bluetoothTransmission.received_data[0].pop(0)
                if len(bluetoothTransmission.received_data[0])==0: 
                    bluetoothTransmission.received_data.pop(0)
        time.sleep(0.005)
        #wait until the prediction is done
        while calibration.new_proprio:
            time.sleep(0.005)
                
    

if __name__ == '__main__':
    calibration.main()
    bluetoothTransmission.main()
    threading.Thread(target=linkBL_and_Calibration).start()