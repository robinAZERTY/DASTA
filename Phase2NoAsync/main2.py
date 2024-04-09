import cv2
import bluetoothTransmission
import time
import numpy as np
import calibration
import progressbar
import sys
# import pygame
import gamecontroller
import FakeDrone

useFakeDrone = False
running_calib = True# not useFakeDrone
quad = FakeDrone.FakeQuad()
bluetoothTransmission.default_bl_address ='FC:F5:C4:27:09:16'
               

received_orientation = [1,0,0,0]
received_position = [0,0,0]
  

"""
in cpp 
enum UserEvent : uint8_t
{
    None=0,
    StartStateEstimate,
    StopStateEstimate,
    StartStream,
    StopStream,
    EnableStateEstimateStream,
    DisableStateEstimateStream,
    EnableSensorStream,
    DisableSensorStream,
    StartGyroBiasEstimation,
    StopGyroBiasEstimation,
    StartAttitudeControl,
    StopAttitudeControl,
    EMERGENCY_STOP
};
"""

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
    calibration.imu.gyr_ortho_std_tol = 0.004
    calibration.imu.acc_ortho_std_tol = 0.004
        
    calibration.init()
    bluetoothTransmission.fake_transmission = useFakeDrone
    bluetoothTransmission.init_transmission('FC:F5:C4:27:09:16')
    
    if not running_calib:
        bluetoothTransmission.data_to_send.append({"user_event": 3}) #StartStream
        bluetoothTransmission.data_to_send.append(
            {
            "gyr_bias_co":[-0.00764382870598456, -0.002906589515953307, -0.008359488215692632],
            "acc_bias_co" :[-0.5381701629683476, -0.16040564811189145, -0.7443503860074567],
            "gyr_ortho_co": [0.9771628295377841, -0.05299494120744518, -0.14232190400465064,
                            0.006196144105722445, 1.0019661037095184, -0.013380961482694914,
                            0.13818732367729306, -0.008985933421890196, 0.9884554847530128],
            "acc_ortho_co": [0.9575774950927878, -0.009070080431738493, -0.13996072437652746,
                            -0.004974358038731178, 1.0127180430455083, -0.03654834981032832,
                            0.13328585931628698, 0.02380419054852441, 0.964911122794868],
            "user_event": 1,#StartStateEstimate
            "send_stream_delay": 50
            })
        bluetoothTransmission.data_to_send.append({"user_event": 11}) #StartAttitudeControl
    else :
        bluetoothTransmission.data_to_send.append({"user_event": 3, "send_stream_delay": 20})
        

                

last_time_calib_indicator = time.time()
calib_indicator_initialized = False
running_calib_indicator = True
gyr_bias_bar = None
acc_bias_bar = None
gyr_ortho_bar = None
acc_ortho_bar = None

def draw_calib_indicator():
    global running_calib_indicator, calib_indicator_initialized, gyr_bias_bar, acc_bias_bar, gyr_ortho_bar, acc_ortho_bar, last_time_calib_indicator
    if not running_calib_indicator:
        return
    if not calib_indicator_initialized:
        gyr_bias_bar = progressbar.ProgressBar(maxval=100, line_offset= 1)
        acc_bias_bar = progressbar.ProgressBar(maxval=100, line_offset= 2)
        gyr_ortho_bar = progressbar.ProgressBar(maxval=100, line_offset= 3)
        acc_ortho_bar = progressbar.ProgressBar(maxval=100, line_offset= 4)
        calib_indicator_initialized = True
        
    t = time.time()
    if t-last_time_calib_indicator < 0.1:
        return
    
    last_time_calib_indicator = t
    
    gyr_bias_bar.update(min(100,int(100*calibration.imu.gyr_bias_std_tol/calibration.imu.gyr_bias_cov_indicator())))
    acc_bias_bar.update(min(100,int(100*calibration.imu.acc_bias_std_tol/calibration.imu.acc_bias_cov_indicator())))
    gyr_ortho_bar.update(min(100,int(100*calibration.imu.gyr_ortho_std_tol/calibration.imu.gyr_ortho_cov_indicator())))
    acc_ortho_bar.update(min(100,int(100*calibration.imu.acc_ortho_std_tol/calibration.imu.acc_ortho_cov_indicator())))
        
    if calibration.imu.calibrated():
        running_calib_indicator = False
        gyr_bias_bar.finish()
        acc_bias_bar.finish()
        gyr_ortho_bar.finish()
        acc_ortho_bar.finish()
        print("gyr_bias, acc_bias, gyr_ortho, acc_ortho")
        print(calibration.imu.gyr_bias_co.tolist(), calibration.imu.acc_bias_co.tolist(), calibration.imu.gyr_ortho_co.reshape(9).tolist(), calibration.imu.acc_ortho_co.reshape(9).tolist())

firstEfkIt = True
new_kalmanIt = False

def on_received_data():
    global running_calib, firstEfkIt, new_kalmanIt
    for buffer in bluetoothTransmission.received_data:
        for data in buffer:
            if data is None:
                continue
            if "orientation" in data:
                gamecontroller.display_attitude = data["orientation"]
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
                    gamecontroller.display_attitude = calibration.criticalState.orientation.elements.tolist()

            if calibration.imu.calibrated() and running_calib:
                to_send = {}
                to_send["gyr_bias_co"] = calibration.imu.gyr_bias_co.tolist()
                to_send["acc_bias_co"] = calibration.imu.acc_bias_co.tolist()
                to_send["gyr_ortho_co"] = calibration.imu.gyr_ortho_co.reshape(9).tolist()
                to_send["acc_ortho_co"] = calibration.imu.acc_ortho_co.reshape(9).tolist()
                to_send["user_event"] = 11 #StartAttitudeControl
                to_send["send_stream_delay"] = 50
                to_send["orientation"] = calibration.criticalState.orientation.elements.tolist()
                running_calib = False
                bluetoothTransmission.data_to_send.append(to_send)
                
    bluetoothTransmission.received_data = []

last_time_run_fake_quad = time.time()
def run_fake_quad(time):
    global last_time_run_fake_quad
    dt = time-last_time_run_fake_quad
    if dt < 0.01:
        return
    last_time_run_fake_quad = time
    quad.run(-gamecontroller.x*0.5,-gamecontroller.y*0.5,-gamecontroller.z,gamecontroller.thrust+gamecontroller.ThrustTilte,dt)
    gamecontroller.display_attitude = quad.state.orientation.elements.tolist()
    gamecontroller.display_position = quad.state.position.tolist()
    received={}
    received["time"] = int(time*1000)
    received["gyro_raw"] = quad.imu.gyr.tolist()
    received["acc_raw"] = quad.imu.acc.tolist()
    received["orientation"] = gamecontroller.display_attitude
    received["position"] = gamecontroller.display_position
    received["w1"] = quad.engines[0].u
    received["w2"] = quad.engines[1].u
    received["w3"] = quad.engines[2].u
    received["w4"] = quad.engines[3].u
    received["angular_velocity_command"] = quad.angular_vel_command.tolist()
    bluetoothTransmission.received_data.append(received)

last_time_pack_command = time.time()
def pack_command():
    global last_time_pack_command
    t = time.time()
    if t-last_time_pack_command < 0.05 or not gamecontroller.is_new_data():
        return
    last_time_pack_command = t
    
    bluetoothTransmission.data_to_send.append({"angular_velocity_command": [gamecontroller.x,gamecontroller.y,gamecontroller.z], "thrust_command": gamecontroller.thrust+gamecontroller.ThrustTilte})

    
if __name__ == "__main__":
    setup()
    while True:
        bluetoothTransmission.run_transmission()
        if len(bluetoothTransmission.received_data) > 0:
            on_received_data()
        gamecontroller.run()
        draw_calib_indicator()
        pack_command()
        if useFakeDrone:
            run_fake_quad(time.time())