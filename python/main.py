import bluetoothTransmission
import calibration
import threading
import time
from enum import Enum
import visu

'''
    wich is in cpp :
        enum UserEvent
            {
                None,
                StartStateEstimate,
                StopStateEstimate,
                StartStream,
                StopStream,
                EnableStateEstimateStream,
                DisableStateEstimateStream,
                EnableSensorStream,
                DisableSensorStream,
            }; 
'''
class UserEvent(Enum):
    NONE = 0
    START_STATE_ESTIMATE = 1
    STOP_STATE_ESTIMATE = 2
    START_STREAM = 3
    STOP_STREAM = 4
    ENABLE_STATE_ESTIMATE_STREAM = 5
    DISABLE_STATE_ESTIMATE_STREAM = 6
    ENABLE_SENSOR_STREAM = 7
    DISABLE_SENSOR_STREAM = 8
    START_GYRO_BIAS_ESTIMATION = 9
    STOP_GYRO_BIAS_ESTIMATION = 10
    


def linkBL_and_Calibration():

    #waite until the bluetooth is connected
    while not bluetoothTransmission.inited:
        time.sleep(0.1)
    
    #do the gyro bias estimation and begin the sensor stream at 40 
    time.sleep(1)
    bluetoothTransmission.data_to_send.append({"user_event": UserEvent.START_GYRO_BIAS_ESTIMATION.value})
    time.sleep(3)
    bluetoothTransmission.data_to_send.append({"user_event": UserEvent.STOP_GYRO_BIAS_ESTIMATION.value})
    bluetoothTransmission.data_to_send.append({"user_event": UserEvent.ENABLE_SENSOR_STREAM.value, "send_stream_delay": 25})
    bluetoothTransmission.data_to_send.append({"user_event": UserEvent.START_STREAM.value})
    while True:
        #check bluetoothTransmission.received_data 
        # print("len(bluetoothTransmission.received_data): ", len(bluetoothTransmission.received_data))
        if len(bluetoothTransmission.received_data) > 0:
            if len(bluetoothTransmission.received_data[0])>0:
                received = bluetoothTransmission.received_data[0][0]
                if received is not None:
                    #check is gyro and accel are in received
                    if "gyro" in received and "acc" in received and "time" in received:
                        # print("received gyro and accel")
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
                
    

def linkCalib_and_Visu():
    #create the visu
    clock = visu.pygame.time.Clock()
    while True:
        # visu.draw_cube(visu.Quaternion(calibration.ekf.Xn[6], calibration.ekf.Xn[7], calibration.ekf.Xn[8], calibration.ekf.Xn[9]))
        visu.draw_Cov(calibration.ekf.Pn)
        # On affiche le résultat
        visu.pygame.display.flip()
        # On attend 10 ms avant de recommencer
        clock.tick(100)
        
    
import matplotlib.pyplot as plt



    
if __name__ == '__main__':
    calibration.main()
    bluetoothTransmission.main()
    threading.Thread(target=linkBL_and_Calibration).start()
    threading.Thread(target=linkCalib_and_Visu).start()
    
    while not bluetoothTransmission.inited:
        time.sleep(0.1)
    time.sleep(60)
    #show the histogram of the calibration.dt_save
    plt.hist(calibration.dt_save, bins=100)
    plt.show()