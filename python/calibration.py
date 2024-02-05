# to use an extended Kalman filter, we can use
import threading
import ekf
import numpy as np
from navpy import angle2quat
from navpy import quat2angle
from navpy import quat2dcm
from navpy import dcm2quat
from pyquaternion import Quaternion
import time as t

'''
____________STATE ESTIMATE VECTOR______________

        State                   unit        index
postion(xyz)                    m           0:2
velocity(xyz)                   m/s         3:5
orientation(quaternion)                     6:9
gyr_bias_correction(xyz)        rad/s       10:12
acc_bias_correction(xyz)        m/s²        13:15
gyr_ortho_correction(3*3)                   16:24
acc_ortho_correction(3*3)                   25:33

.............camera parameters................
cam_0_pos(xyz)                  m           34:36
cam_0_ori(quaternion)                       37:40
cam_0_k                         pixel       41
                 ...
cam_m_pos(xyz)                  m           34+8*m:36+8*m
cam_m_ori(quaternion)                       37+8*m:40+8*m
cam_m_k                         pixel       41+8*m


___________COMMANDS VECTOR_____________________

        State                   unit        index
gyroscopes(xyz)                 rad/s       1:3
accelerometers(xyz)             m/s²        4:6


___________SENSOR VECTORS_____________________

        State                   unit       index
beac1_cam1(uv)                  pixel       0:1

'''           
class criticalState:
        def __init__(self,position,velocity, orientation):
                self.position = position
                self.velocity = velocity
                self.orientation = orientation
                
                self.position_cov = np.eye(3)
                self.velocity_cov = np.eye(3)
                self.orientation_cov = np.eye(4)*0.1
                
        def __str__(self) -> str:
                return "criticalState\nposition = "+str(self.position)+"\nvelocity = "+str(self.velocity)+"\norientation = "+str(self.orientation)+"\nposition_cov = "+str(self.position_cov)+"\nvelocity_cov = "+str(self.velocity_cov)+"\norientation_cov = "+str(self.orientation_cov)
                
                
class MPU9250:
        def __init__(self,gyr_noise,gyr_bias_tolerance,gyr_cross_tolerance,acc_noise,acc_xy_bias_tolerance,acc_z_bias_tolerance,acc_gain_tolerance,acc_cross_tolerance):               
                self.new_acc_sample = None
                self.new_gyr_sample = None
                self.time = None
                self.prev_time = None
                
                self.gyr_noise = np.eye(3)*gyr_noise**2
                self.acc_noise = np.eye(3)*acc_noise**2
                
                self.acc_bias_co = np.zeros(3)
                self.gyr_bias_co = np.zeros(3)
                
                self.gyr_ortho_co = np.eye(3)
                self.acc_ortho_co = np.eye(3)

                self.gyr_bias_cov = np.eye(3)*gyr_bias_tolerance**2
                self.acc_bias_cov = np.eye(3)*acc_xy_bias_tolerance**2
                self.acc_bias_cov[2,2] = acc_z_bias_tolerance**2
                
                self.acc_ortho_cov = np.eye(9)*acc_cross_tolerance**2
                self.gyr_ortho_cov = np.eye(9)*gyr_cross_tolerance**2
                
        def __str__(self) -> str:
                return "MPU9250\ngyr_noise = "+str(self.gyr_noise)+"\nacc_noise = "+str(self.acc_noise)+"\nacc_bias_co = "+str(self.acc_bias_co)+"\ngyr_bias_co = "+str(self.gyr_bias_co)+"\nacc_ortho_co = "+str(self.acc_ortho_co)+"\ngyr_ortho_co = "+str(self.gyr_ortho_co)+"\nacc_bias_cov = "+str(self.acc_bias_cov)+"\ngyr_bias_cov = "+str(self.gyr_bias_cov)+"\nacc_ortho_cov = "+str(self.acc_ortho_cov)+"\ngyr_ortho_cov = "+str(self.gyr_ortho_cov)
                           

class led:
        def __init__(self,mounting_position):
                self.mounting_position = mounting_position
                
        def __str__(self) -> str:
                return "led\nmounting_position = "+str(self.mounting_position)
                
class camera:
        def __init__(self,position, orientation, k,  pos_accur, ori_accur, k_accur, noise=1):
                self.position = position
                self.orientation = orientation
                self.k = k
                self.pos_cov = np.eye(3)*pos_accur**2
                self.ori_cov = np.eye(4)*ori_accur**2
                self.k_cov= k_accur**2
                self.noise = noise
                
                self.led_measurements = None
                self.led_measurement_cov = np.eye(2)*noise**2
                
        def __str__(self) -> str:
                return "camera\nposition = "+str(self.position)+"\norientation = "+str(self.orientation)+"\nk = "+str(self.k)+"\npos_cov = "+str(self.pos_cov)+"\nori_cov = "+str(self.ori_cov)+"\nk_cov = "+str(self.k_cov)+"\nnoise = "+str(self.noise)+"\nled_measurements = "+str(self.led_measurements)+"\nled_measurement_cov = "+str(self.led_measurement_cov)

class environment:
        def __init__(self, gravity):
                self.gravity = gravity
                
        def __str__(self) -> str:
                return "environment\ngravity = "+str(self.gravity)
                

env = environment(9.812)
criticalState = criticalState(np.array([0,0,0]),np.array([0,0,0]),Quaternion(1,0,0,0))
     
#defaut mpu6050 parameters
gyr_noise = 0.1*np.pi/180
gyr_bias_tolerance = 0.03
gyr_cross_tolerance = 0.02
acc_noise = 0.008*9.81
acc_xy_bias_tolerance = 0.06*9.81
acc_z_bias_tolerance = 0.08*9.81
acc_gain_tolerance = 0.03
acc_cross_tolerance = 0.03

imu = MPU9250(gyr_noise,gyr_bias_tolerance,gyr_cross_tolerance,acc_noise,acc_xy_bias_tolerance,acc_z_bias_tolerance,acc_gain_tolerance,acc_cross_tolerance)

             
#defaut mounting camera parameters
leds = [led(np.array([0.1,0,-0.1])),led(np.array([-0.1,0,-0.1]))]

# cam_res = 800
# cam_AoV = 135*np.pi/180
# default_cam_k = cam_res/(2*np.tan(cam_AoV/2))
default_cam_k = 800/3
cam_AoV_accur = 0.00
cam_mounting_accur_pos = 0
cam_mounting_accur_ori = 0

# k_uncertainty = 0.5*(cam_res/(2*np.tan((cam_AoV+cam_AoV_accur)/2))-cam_res/(2*np.tan((cam_AoV-cam_AoV_accur)/2)))
k_uncertainty = 1
cam1_position = np.array([0,0,-0.5])
q0,qvec=angle2quat(0,0,0)
cam1_orien = [q0,qvec[0],qvec[1],qvec[2]]
cam_noise = 1

cams = [camera(cam1_position,cam1_orien,default_cam_k,cam_mounting_accur_pos,cam_mounting_accur_ori,k_uncertainty, cam_noise)]




def transition_function(X,U,dt_imu,g):
        '''
        transition function
        '''
        xyz = X[0:3]
        vxyz = X[3:6]
        q0,qvec = X[6],X[7:10]
        gbxyz = X[10:13]
        abxyz = X[13:16]
        gyro_o = X[16:25].reshape(3,3)
        acc_o = X[25:34].reshape(3,3)
        
        gyro = U[0:3]
        acc= U[3:6]
        
        #correction of gyro and acc
        gyro = gyro_o@(gyro+gbxyz)

        acc = acc_o@(acc+abxyz)    
        # integration of linear velocities
        xyz_ = xyz + vxyz*dt_imu
        
        # integration of linear accelerations
        # print("acc = ",acc)
        qdcm = quat2dcm(q0,-qvec)
        ae = qdcm@acc
        ae[2] = ae[2] + g
        vxyz_ = vxyz + ae*dt_imu
        q_ = Quaternion(q0,qvec[0],qvec[1],qvec[2])
        q_.integrate(gyro,dt_imu)
        q_ = q_.elements
        
        newX = X.copy()
        newX[0:3] = xyz_
        newX[3:6] = vxyz_
        newX[6:10] = q_[0:4]
        
        return newX


def project(led_pos,drone_orien,drone_pos,cam_pos,cam_orien,cam_k):
        '''
        project a point from world frame to camera frame
        '''
        qdcm = quat2dcm(drone_orien[0],-drone_orien[1:4])
        res = qdcm@led_pos
        res1 = res + drone_pos - cam_pos
        qdcm2 = quat2dcm(cam_orien[0],-cam_orien[1:4])
        res2 = qdcm2@res1
        res3 = res2[0:2]*cam_k/res2[2]
        return res3

def hcmln(X, m, n):
        drone_orien = X[6:10].copy()
        drone_pos = X[0:3].copy()
        led_pos = leds[n].mounting_position.copy()
        cam_pos = X[m*8+34:m*8+37].copy()
        cam_orien = X[m*8+37:m*8+41].copy()
        k = X[41+m*8].copy()
        return project(led_pos,drone_orien,drone_pos,cam_pos,cam_orien,k)


def calib2X(Q, x, P, criState, imu, cams):
        '''
        link object to Q, x and P
        '''
        if (Q is None):
                Q = np.zeros((6,6))
                
        Q[0:3,0:3] = imu.gyr_noise
        Q[3:6,3:6] = imu.acc_noise
        

        if (x is None):
                x = np.zeros(34+8*len(cams))
                
        x[0:3] = criState.position
        x[3:6] = criState.velocity
        x[6:10] = criState.orientation.elements
        x[10:13] = imu.gyr_bias_co
        x[13:16] = imu.acc_bias_co
        x[16:25] = imu.gyr_ortho_co.reshape(9)
        x[25:34] = imu.acc_ortho_co.reshape(9)

        for i in range(len(cams)):
                x[34+8*i:37+8*i] = cams[i].position
                x[37+8*i:41+8*i] = cams[i].orientation
                x[41+8*i] = cams[i].k
                
        if (P is None):
                P = np.zeros((x.shape[0],x.shape[0]))
                
        P[0:3,0:3] = criState.position_cov
        P[3:6,3:6] = criState.velocity_cov
        P[6:10,6:10] = criState.orientation_cov
        P[10:13,10:13] = imu.gyr_bias_cov
        P[13:16,13:16] = imu.acc_bias_cov
        P[16:25,16:25] = imu.gyr_ortho_cov
        P[25:34,25:34] = imu.acc_ortho_cov
        for i in range(len(cams)):
                P[34+8*i:37+8*i,34+8*i:37+8*i] = cams[i].pos_cov
                P[37+8*i:41+8*i,37+8*i:41+8*i] = cams[i].ori_cov
                P[41+8*i,41+8*i] = cams[i].k_cov
                
        return Q,x,P

def X2calib(X,P, criState, imu, cams):
        '''
        link state vector to object
        '''
        criState.position = X[0:3]
        criState.velocity = X[3:6]
        criState.orientation = Quaternion(X[6],X[7],X[8],X[9])
        imu.gyr_bias_co = X[10:13]
        imu.acc_bias_co = X[13:16]
        imu.gyr_ortho_co = X[16:25].reshape(3,3)
        imu.acc_ortho_co = X[25:34].reshape(3,3)
        
        for i in range(len(cams)):
                cams[i].position = X[34+8*i:37+8*i]
                cams[i].orientation = Quaternion(X[37+8*i:41+8*i])
                cams[i].k = X[41+8*i]
                
        criState.position_cov = P[0:3,0:3]
        criState.velocity_cov = P[3:6,3:6]
        criState.orientation_cov = P[6:10,6:10]
        
        for i in range(len(cams)):
                cams[i].pos_cov = P[34+8*i:37+8*i,34+8*i:37+8*i]
                cams[i].ori_cov = P[37+8*i:41+8*i,37+8*i:41+8*i]
                cams[i].k_cov = P[41+8*i,41+8*i]
                
        return criState, imu, cams

my_ekf = ekf.Ekf(34+8*len(cams))
Q = None
      
def init():
        global Q, my_ekf
        Q, my_ekf.x, my_ekf.P = calib2X(Q, my_ekf.x, my_ekf.P, criticalState, imu, cams)


def predict():
        '''
        predict the state
        '''
        global criticalState, imu, cams
                
        if imu.new_gyr_sample is None or imu.new_acc_sample is None:
                return
        
        if imu.prev_time is None:
                imu.prev_time = imu.time
                return
        
        u = np.concatenate((imu.new_gyr_sample,imu.new_acc_sample))
        f = lambda x,u: transition_function(x,u,imu.time - imu.prev_time,env.gravity)
        my_ekf.predict(f,u,Q)
        imu.prev_time = imu.time
        imu.new_gyr_sample = None
        imu.new_acc_sample = None

        criticalState, imu, cams = X2calib(my_ekf.x, my_ekf.P, criticalState, imu, cams)



def update(force_center = None):
        '''
        update the state using measurements
        '''
        global criticalState, imu, cams
        
        if force_center is not None:
                for m in range(len(cams)):
                        cams[m].led_measurements = []
                        for n in range(len(leds)):
                                xx = my_ekf.x.copy()
                                xx[0:3] = 0
                                z = hcmln(xx,m,n) + np.random.normal(0,cams[m].noise,2)
                                cams[m].led_measurements.append(z)
                                
                                my_ekf.update(lambda x: hcmln(x,m,n),z,cams[m].led_measurement_cov*9)
                                
   
        
        for cam in cams:
                if cam.led_measurements is not None:
                       # determiner l'appartenance de chaque mesure a une led
                       pass

         
        criticalState, imu, cams = X2calib(my_ekf.x, my_ekf.P, criticalState, imu, cams)        