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
running_calib = False# not useFakeDrone
quad = FakeDrone.FakeQuad()
bluetoothTransmission.default_bl_address ='24:6F:28:7B:DB:22'
bluetoothTransmission.print_reception = False
bluetoothTransmission.print_transmission = False           

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
    calibration.imu.gyr_ortho_std_tol = 0.005
    calibration.imu.acc_ortho_std_tol = 0.005
  
    # calibration.imu.gyr_bias_std_tol *= 2
    # calibration.imu.acc_bias_std_tol *= 2
    # calibration.imu.gyr_ortho_std_tol *= 2
    # calibration.imu.acc_ortho_std_tol *= 2

      
    calibration.init()
    bluetoothTransmission.fake_transmission = useFakeDrone
    bluetoothTransmission.init_transmission('08:D1:F9:CE:C3:76')
    
    if not running_calib:
        bluetoothTransmission.data_to_send.append({"user_event": 3}) #StartStream
        bluetoothTransmission.data_to_send.append({
            'gyro_bias_co': [-0.11896539564419907, 0.012694509160569165, -0.015780466117347472],
            'acc_bias_co': [-0.0560912679013639, 0.1483948803669653, -0.5406994399014228],
            'gyro_scale_co': [[1.0095270720940326, -0.006502963491368057, -0.0009163382881876442],
                              [0.011263640184387785, 1.001754653043537, 0.009827374124837161],
                              [-0.006914491274247384, 0.007181100172026654, 0.998832585708012]],
            'acc_scale_co': [[0.9921890274118018, -0.0083805195897565, 0.00795622439258923],
                             [0.007551945246112519, 0.9987173123942706, 0.009975625312003923],
                             [-0.0005677593159086044, -0.007056947631665735, 0.9910633866799974]],
            'user_event': 1, 'send_stream_delay': 50})
        bluetoothTransmission.data_to_send.append({"user_event": 11}) #StartAttitudeControl
    else :
        bluetoothTransmission.data_to_send.append({"user_event": 3, "send_stream_delay": 20})
        

                

last_time_calib_indicator = time.time()
calib_indicator_initialized = False
running_calib_indicator = running_calib
gyr_bias_bar = None
acc_bias_bar = None
gyr_ortho_bar = None
acc_ortho_bar = None

display_3d = True
battery = 0
battery_voltage = 0
def run3d():
    global display_3d, battery
    if not display_3d:
        return
    gamecontroller.clock.tick(gamecontroller.FPS)
    for event in gamecontroller.pygame.event.get():
        if event.type == gamecontroller.pygame.JOYDEVICEADDED:
            joy = gamecontroller.pygame.joystick.Joystick(event.device_index)
            gamecontroller.joysticks.append(joy)
        #quit program
        if event.type == gamecontroller.pygame.QUIT:
            print("QUITTING")
            display_3d = False
            gamecontroller.pygame.quit()
    if display_3d:
        gamecontroller.screen.fill(gamecontroller.pygame.Color("midnightblue"))
        if useFakeDrone:
            gamecontroller.draw_box(quad.state.orientation.elements.tolist(),gamecontroller.display_position,(0,255,0))
        gamecontroller.draw_box()
        #add battery level
        gamecontroller.pygame.draw.rect(gamecontroller.screen, gamecontroller.pygame.Color("black"), gamecontroller.pygame.Rect(0, 0, 100, 20))
        gamecontroller.pygame.draw.rect(gamecontroller.screen, gamecontroller.pygame.Color("red"), gamecontroller.pygame.Rect(0, 0, battery, 20))
        # display battery voltage with text
        font = gamecontroller.pygame.font.Font(None, 20)
        text = font.render(str(round(battery_voltage,1)), True, gamecontroller.pygame.Color("white"))
        gamecontroller.screen.blit(text, (110, 5))
        
        gamecontroller.pygame.display.flip()


last_calibration = {}
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
        print("calibration done :")
        print(last_calibration)
    
firstEfkIt = True
new_kalmanIt = False

'''
enum EmbeddedEvent : uint8_t
{
    None2=0,
    WaitingForBatteryPower,
    WaitingForCalibration,
    WaitingForKalmanConvergence,
    ReadyToFly,
    LowBattery,
};
'''
def on_received_data():
    global running_calib, firstEfkIt, new_kalmanIt, battery, battery_voltage
    for buffer in bluetoothTransmission.received_data:
        for data in buffer:
            if data is None:
                continue
            # print("received data: " + str(data))
            to_send = {}
            if "orientation" in data:
                gamecontroller.display_attitude = data["orientation"]
                # print("received orientation: " + str(data["orientation"]))
            if "time" in data and "gyro_raw" in data and "acc_raw" in data:
                calibration.imu.time = data["time"]/1000
                calibration.imu.new_gyr_sample = np.array(data["gyro_raw"])
                calibration.imu.new_acc_sample = np.array(data["acc_raw"])
                if firstEfkIt:
                    calibration.initAttitudeUsingAcc()
                    firstEfkIt = False
                elif running_calib:
                    calibration.predict()
                    calibration.update()
                    new_kalmanIt = True
                    gamecontroller.display_attitude = calibration.criticalState.orientation.elements.tolist()
            if "internal_event" in data:
                if data["internal_event"] == 1:
                    print("Waiting for battery power")
                elif data["internal_event"] == 2:
                    print("Waiting for calibration")
                elif data["internal_event"] == 3:
                    print("Waiting for kalman convergence")
                elif data["internal_event"] == 4:
                    print("Ready to fly")
                elif data["internal_event"] == 5:
                    print("Low battery")
            if "battery_lvl" in data:
                battery = data["battery_lvl"][-1]*100
            if "battery_voltages" in data:
                battery_voltage = data["battery_voltages"][-1]
                
 

            if calibration.imu.calibrated() and running_calib:
                global last_calibration
                to_send["gyro_bias_co"] = calibration.imu.gyr_bias_co.tolist()
                to_send["acc_bias_co"] = calibration.imu.acc_bias_co.tolist()
                # to_send["gyro_scale_co"] = calibration.imu.gyr_ortho_co.reshape(9).tolist()
                # to_send["acc_scale_co"] = calibration.imu.acc_ortho_co.reshape(9).tolist()
                # to_send["gyro_scale_co"] should be a list of 3 list of 3 elements
                to_send["gyro_scale_co"] = calibration.imu.gyr_ortho_co.reshape(3,3).tolist()
                to_send["acc_scale_co"] = calibration.imu.acc_ortho_co.reshape(3,3).tolist()
                to_send["user_event"] = 11 #StartAttitudeControl
                to_send["send_stream_delay"] = 50
                last_calibration = to_send
                running_calib = False
                print("calibration done :" + str(to_send))
                bluetoothTransmission.data_to_send.append(to_send)
                bluetoothTransmission.data_to_send.append({"user_event": 1}) #StartStateEstimate
    # print("received data: " + str(bluetoothTransmission.received_data))  
    bluetoothTransmission.received_data = []

last_time_run_fake_quad = time.time()
def run_fake_quad(time):
    quad.receive(bluetoothTransmission.data_to_send)
    quad.send(bluetoothTransmission.received_data)

    global last_time_run_fake_quad
    dt = time-last_time_run_fake_quad
    if dt < 0.01:
        return
    last_time_run_fake_quad = time
    if running_calib:
        quad.force_angular_vel(-gamecontroller.x*2,-gamecontroller.y*2,-gamecontroller.z*2,dt)
    else:
        quad.run(-gamecontroller.x*0.1,-gamecontroller.y*0.1,-gamecontroller.z,gamecontroller.thrust+gamecontroller.ThrustTilte,dt)
    # gamecontroller.display_attitude = quad.state.orientation.elements.tolist()
    gamecontroller.display_position = quad.state.position.tolist()


last_time_pack_command = time.time()
def pack_command():
    global last_time_pack_command
    t = time.time()
    if t-last_time_pack_command < 0.05 or not gamecontroller.is_new_data():
        return
    last_time_pack_command = t
    
    # bluetoothTransmission.data_to_send.append({"angular_velocity_command": [gamecontroller.x,gamecontroller.y,gamecontroller.z], "thrust_command": gamecontroller.thrust+gamecontroller.ThrustTilte})
    bluetoothTransmission.data_to_send.append({"rpy_command": [gamecontroller.x*0.1,gamecontroller.y*0.1,0], "angular_velocity_command": [0,0,gamecontroller.z],"thrust_command": gamecontroller.thrust+gamecontroller.ThrustTilte})
    if (gamecontroller.engage):
        bluetoothTransmission.data_to_send.append({"user_event" : 14})
        gamecontroller.engage = False
    if (gamecontroller.disengage):
        bluetoothTransmission.data_to_send.append({"user_event" : 15})
        gamecontroller.disengage = False

        

    
if __name__ == "__main__":
    setup()
    while True:
        if len(bluetoothTransmission.received_data) > 0:
            on_received_data()
        # gamecontroller.run()
        run3d()
        gamecontroller.run_controller()
        if gamecontroller.emergency_stop:
            print("EMERGENCY STOP")
            bluetoothTransmission.data_to_send.append({"user_event": 13})
            gamecontroller.emergency_stop = False
        draw_calib_indicator()
        pack_command()
        if not useFakeDrone:
            bluetoothTransmission.run_transmission()
        else:
            run_fake_quad(time.time())