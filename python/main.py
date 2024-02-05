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
    


def linkBL_and_Calibration():

    calibration.init()
    #waite until the bluetooth is connected
    while not bluetoothTransmission.inited:
        time.sleep(0.1)
    
    time.sleep(1)
    bluetoothTransmission.data_to_send.append({"user_event": UserEvent.ENABLE_SENSOR_STREAM.value, "send_stream_delay": 20})
    bluetoothTransmission.data_to_send.append({"user_event": UserEvent.START_STREAM.value})
    while True:
        #check bluetoothTransmission.received_data 
        # print("len(bluetoothTransmission.received_data): ", len(bluetoothTransmission.received_data))
        for buffer in bluetoothTransmission.received_data:
            for data in buffer:
                if data is None:
                    continue
                if "gyro" in data and "acc" in data and "time" in data:
                    calibration.imu.time = data["time"]/1000
                    calibration.imu.new_gyr_sample = data["gyro"]
                    calibration.imu.new_acc_sample = data["acc"]
                    calibration.predict()
                    calibration.update(0.1)
                    # print(calibration.cams[0].led_measurements)
        bluetoothTransmission.received_data = []
        
        # if len(bluetoothTransmission.received_data) > 0:
        #     print(bluetoothTransmission.received_data)
        #     if len(bluetoothTransmission.received_data[0])>0:
        #         received = bluetoothTransmission.received_data[0][0]
        #         if received is not None:
        #             #check is gyro and accel are in received
        #             if "gyro" in received and "acc" in received and "time" in received:
        #                 # print("received gyro and accel")
        #                 calibration.imu.time = received["time"]/1000
        #                 calibration.imu.new_gyr_sample = received["gyro"]
        #                 calibration.imu.new_acc_sample = received["acc"]
                        # if calibration.imu.prev_time is not None:
                            # print(calibration.imu.time-calibration.imu.prev_time)
        
            # if len(bluetoothTransmission.received_data)>0:
            #     if len(bluetoothTransmission.received_data[0])>0:
            #         bluetoothTransmission.received_data[0].pop(0)
            #     if len(bluetoothTransmission.received_data[0])==0: 
            #         bluetoothTransmission.received_data.pop(0)
        time.sleep(0.001)                
    

def linkCalib_and_Visu():
    #create the visu
    clock = visu.pygame.time.Clock()
    while True:
        # visu.draw_cube(visu.Quaternion(calibration.ekf.Xn[6], calibration.ekf.Xn[7], calibration.ekf.Xn[8], calibration.ekf.Xn[9]), calibration.ekf.Xn[0:3].reshape(3))
        # visu.draw_Cov(calibration.ekf.Pn)
        # visu.draw_state(calibration.ekf.Xn)
        visu.draw_cube(calibration.criticalState.position, calibration.criticalState.orientation, calibration.cams[0].led_measurements)
        # On affiche le résultat
        visu.pygame.display.flip()
        # On attend 10 ms avant de recommencer
        clock.tick(100)

        
    
import matplotlib.pyplot as plt



    
if __name__ == '__main__':
    # calibration.main(0.1)
    bluetoothTransmission.main()
    threading.Thread(target=linkBL_and_Calibration).start()
    threading.Thread(target=linkCalib_and_Visu).start()
    
    # while not bluetoothTransmission.inited:
    #     time.sleep(0.1)
    # time.sleep(60)
    # #show the histogram of the calibration.dt_save
    # plt.hist(calibration.dt_save, bins=100)
    # plt.show()