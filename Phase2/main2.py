import cv2
import bluetoothTransmission
import time
import numpy as np
import threading
import calibration
import progressbar
import sys

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
        projected.append(calibration.project(vertex,np.array(received_orientation),np.array([0,0,0]),np.array([-0.2,0,-0.03]),np.array([ 0.5, 0.5, 0.5, 0.5 ]),300))
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
    calibration.imu.setGyrOrthoCov(0.01,0.01)
    calibration.imu.setAccNoise(0.1)
    calibration.imu.setAccBiasCov(0.7848)
    calibration.imu.setAccOrthoCov(0.01,0.01)
    
    calibration.imu.gyr_bias_std_tol = 0.0003
    calibration.imu.acc_bias_std_tol = 0.005
    calibration.imu.gyr_ortho_std_tol = 0.003
    calibration.imu.acc_ortho_std_tol = 0.003
        
    calibration.init()


def draw_attitude():
    # while True:
    img = np.zeros((300,300,3), np.uint8)
    # add indicator of calibration state using calibration.imu.gyr_bias_calibrated(), calibration.imu.acc_bias_calibrated(), calibration.imu.gyr_ortho_calibrated(), calibration.imu.acc_ortho_calibrated()
    cv2.putText(img, "GyrBiasCalibrated: " + str(calibration.imu.gyr_bias_calibrated()), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, "AccBiasCalibrated: " + str(calibration.imu.acc_bias_calibrated()), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, "GyrOrthoCalibrated: " + str(calibration.imu.gyr_ortho_calibrated()), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, "AccOrthoCalibrated: " + str(calibration.imu.acc_ortho_calibrated()), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    draw_box(img) 
    cv2.imshow("Box", img)

running_calib = True

def draw():
    while not bluetoothTransmission.inited:
        cv2.waitKey(1)

    print("gyr_bias, acc_bias, gyr_ortho, acc_ortho")
    print("\n\n\n\n")
    gyr_bias_bar = progressbar.ProgressBar(maxval=100, line_offset= 1)
    acc_bias_bar = progressbar.ProgressBar(maxval=100, line_offset= 2)
    gyr_ortho_bar = progressbar.ProgressBar(maxval=100, line_offset= 3)
    acc_ortho_bar = progressbar.ProgressBar(maxval=100, line_offset= 4)
    # Create a file descriptor for regular printing as well
    print_fd = progressbar.LineOffsetStreamWrapper(lines=0, stream=sys.stdout)
    assert print_fd

    
    while True:
        draw_attitude()
        time.sleep(0.05)
        gyr_bias_bar.update(min(100,int(100*calibration.imu.gyr_bias_std_tol/calibration.imu.gyr_bias_cov_indicator())))
        acc_bias_bar.update(min(100,int(100*calibration.imu.acc_bias_std_tol/calibration.imu.acc_bias_cov_indicator())))
        gyr_ortho_bar.update(min(100,int(100*calibration.imu.gyr_ortho_std_tol/calibration.imu.gyr_ortho_cov_indicator())))
        acc_ortho_bar.update(min(100,int(100*calibration.imu.acc_ortho_std_tol/calibration.imu.acc_ortho_cov_indicator())))

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
    
    bluetoothTransmission.data_to_send.append({"user_event": 13}) #EMERGENCY_STOP
    gyr_bias_bar.finish()
    acc_bias_bar.finish()
    gyr_ortho_bar.finish()
    acc_ortho_bar.finish()
    cv2.destroyAllWindows()

        
def sendCommand():
    #buid the rpy command from game controller and send it
    pass

def main():
    global received_orientation, running_calib
    setup()
    firstEfkIt = True
    while not bluetoothTransmission.inited:
        cv2.waitKey(1)


    bluetoothTransmission.data_to_send.append({"send_stream_delay": 10, "user_event": 3})
    while(True):
        new_kalmanIt = False
        for buffer in bluetoothTransmission.received_data:
            for data in buffer:
                # print(data)
                if data is None:
                    continue
                if "orientation" in data:
                    received_orientation = data["orientation"]
                if "time" in data and "gyro_raw" in data and "acc_raw" in data:
                    calibration.imu.time = data["time"]/1000
                    calibration.imu.new_gyr_sample = np.array(data["gyro_raw"])
                    calibration.imu.new_acc_sample = np.array(data["acc_raw"])
                    if firstEfkIt:
                        calibration.initAttitudeUsingAcc()
                        firstEfkIt = False
                    if running_calib:
                        calibration.predict()
                        calibration.update()
                        new_kalmanIt = True
                if calibration.imu.calibrated() and running_calib:
                    bluetoothTransmission.data_to_send.append({"gyr_bias_co": calibration.imu.gyr_bias_co.tolist(), "acc_bias_co": calibration.imu.acc_bias_co.tolist(), "gyr_ortho_co": calibration.imu.gyr_ortho_co.reshape(9).tolist(), "orientation" : calibration.criticalState.orientation.elements.tolist(),"acc_ortho_co": calibration.imu.acc_ortho_co.reshape(9).tolist(), "user_event": 11, "send_stream_delay":50}) #SEND_CALIBRATION and START_ATTITUDE_CONTROL
                    running_calib = False


        # if new_kalmanIt:
        #     bluetoothTransmission.data_to_send.append({"orientation": calibration.criticalState.orientation.elements.tolist()}) #SEND_ATTITUDE
        bluetoothTransmission.received_data = []        #close the window
        time.sleep(0.001)
    
if __name__ == "__main__":
    bluetoothTransmission.main()#init the bluetooth
    threading.Thread(target=main).start()
    threading.Thread(target=draw).start()

'''
sending : {
'gyr_bias_co': [0.002801945970295527, 0.029011749265692822, 0.006528320068460152], 
'acc_bias_co': [0.051844492672723085, 0.25888215285081284, -0.16931503135376746], 
'gyr_ortho_co': [1.0001186968305638, 0.0035822877831092215, 0.05601770609824901,
                -0.01090682160310374, 0.9960287895051348, 0.020462629344681432, 
                -0.05782902041304632, 0.012969463821746484, 1.008285422856452], 
'acc_ortho_co': [0.9713035146466907, 0.011343414721059332, 0.0550657136657784, 
                0.007858472466955618, 0.9876143664476156, 0.0006895721787651755, 
                -0.06374875003554434, -0.014924327347036194, 0.9891866024945776], 
'user_event': 11, 'send_stream_delay': 50}

sending : {
'gyr_bias_co': [0.001480111875270763, 0.027379781436429575, 0.003728147959764632], 
'acc_bias_co': [-0.09066368609265504, 0.05678553569631716, -0.16818117885522724], 
'gyr_ortho_co': [0.9857398902753511, 0.1766693293253466, -0.012736265612045434, 
                -0.16526740325914183, 0.9807870201258838, -0.15380944713849254, 
                -0.029374779774986957, 0.17748753724523122, 0.9713055867862417], 
'acc_ortho_co': [0.9688797199187668, 0.17432720253430323, 0.02204680192852682, 
                -0.1620987331318013, 0.9792023908415275, -0.1450348774493668, 
                -0.01658107019549306, 0.1461910747069092, 0.9968784020748184], 
'user_event': 11, 'send_stream_delay': 50}
'''