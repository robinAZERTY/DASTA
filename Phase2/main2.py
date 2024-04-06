import cv2
import bluetoothTransmission
import time
import numpy as np
import threading
import calibration

received_orientation = [1,0,0,0]
def draw_box(img):
    #project the box on the image using the calibration.prject function
    dx = 0.07
    dy = 0.055
    dz = 0.035
    vertices = np.array([[-dx/2,-dy/2,-dz/2],
                            [dx/2,-dy/2,-dz/2],
                            [dx/2,dy/2,-dz/2],
                            [-dx/2,dy/2,-dz/2],
                            [-dx/2,-dy/2,dz/2],
                            [dx/2,-dy/2,dz/2],
                            [dx/2,dy/2,dz/2],
                            [-dx/2,dy/2,dz/2]])
    #project the vertices
    projected = []
    for vertex in vertices:
        projected.append(calibration.project(vertex,calibration.criticalState.orientation.elements,np.array([0,0,0]),np.array([-0.3,0,0]),np.array([ 0.5, 0.5, 0.5, 0.5 ]),300))
        # projected.append(calibration.hcmln(calibration.my_ekf.x,0,vertex))
    # print(projected[0], img.shape)
    #draw the lines
    for i, j in [(0, 1), (1, 2), (2, 3), (3, 0),
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7)]:
            cv2.line(img,
                            (round(projected[i][0]+img.shape[1]/2),
                            round(projected[i][1]+img.shape[0]/2)),
                            (round(projected[j][0]+img.shape[1]/2),
                            round(projected[j][1]+img.shape[0]/2)), (255,255,255), 1)


def setup():
    calibration.env= calibration.environment()
    calibration.imu = calibration.MPU9250()
    calibration.criticalState = calibration.CriticalState()
    calibration.env.gravity = 9.812
    q0, qvec = calibration.angle2quat(0,0,0, input_unit='deg')
    calibration.criticalState.orientation = calibration.Quaternion(q0,qvec[0],qvec[1],qvec[2])
    calibration.criticalState.setOriCov(5/180.0*3.14)
    
    calibration.imu.setGyrNoise(0.0017)
    calibration.imu.setGyrBiasCov(0.03)
    calibration.imu.setGyrOrthoCov(0.02,0.03)
    calibration.imu.setAccNoise(0.0785)
    calibration.imu.setAccBiasCov(0.7848)
    calibration.imu.setAccOrthoCov(0.03,0.03)
    calibration.init()


def draw_attitude():
    
    img = np.zeros((300,300,3), np.uint8)
    
    # add indicator of calibration state using calibration.imu.gyr_bias_calibrated(), calibration.imu.acc_bias_calibrated(), calibration.imu.gyr_ortho_calibrated(), calibration.imu.acc_ortho_calibrated()
    cv2.putText(img, "GyrBiasCalibrated: " + str(calibration.imu.gyr_bias_calibrated()), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, "AccBiasCalibrated: " + str(calibration.imu.acc_bias_calibrated()), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, "GyrOrthoCalibrated: " + str(calibration.imu.gyr_ortho_calibrated()), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, "AccOrthoCalibrated: " + str(calibration.imu.acc_ortho_calibrated()), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    if calibration.imu.calibrated() and running_calib:
        bluetoothTransmission.data_to_send.append({"gyr_bias_co": calibration.imu.gyr_bias_co.tolist(), "acc_bias_co": calibration.imu.acc_bias_co.tolist(), "gyr_ortho_co": calibration.imu.gyr_ortho_co.reshape(9).tolist(), "acc_ortho_co": calibration.imu.acc_ortho_co.reshape(9).tolist(), "user_event": 11}) #SEND_CALIBRATION and START_ATTITUDE_CONTROL
        running_calib = False
    draw_box(img) 
    cv2.imshow("Box", img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        cv2.destroyAllWindows()
        #emergency stop
        bluetoothTransmission.data_to_send.append({"user_event": 13}) #EMERGENCY_STOP
        


def main():
    global received_orientation
    setup()
    firstEfkIt = True
    running_calib = True
    while not bluetoothTransmission.inited:
        cv2.waitKey(1)
        
    time.sleep(1)
    bluetoothTransmission.data_to_send.append({"send_stream_delay": 50, "user_event": 3})

    running_calib = False
    while(True):
        for buffer in bluetoothTransmission.received_data:
            for data in buffer:
                # print(qqdata)
                if data is None:
                    continue
                if "time" in data and "gyro_raw" in data and "acc_raw" in data:
                    calibration.imu.time = data["time"]/1000
                    calibration.imu.new_gyr_sample = np.array(data["gyro_raw"])
                    calibration.imu.new_acc_sample = np.array(data["acc_raw"])
                    if firstEfkIt:
                        calibration.initAttitudeFromAcc()
                        firstEfkIt = False
                    if running_calib:
                        calibration.predict()
                        calibration.update()
                        
                    bluetoothTransmission.data_to_send.append({"orientation": calibration.criticalState.orientation.elements.tolist()}) #SEND_ATTITUDE

                if "orientation" in data:
                    received_orientation = data["orientation"]
                
                        
        bluetoothTransmission.received_data = []        #close the window
        
if __name__ == "__main__":
    bluetoothTransmission.main()#init the bluetooth
    threading.Thread(target=main).start()