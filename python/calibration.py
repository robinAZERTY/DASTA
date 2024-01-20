# to use an extended Kalman filter, we can use
import threading
import myKalmanFilter as kf
import numpy as np
from navpy import angle2quat
from navpy import quat2angle
from navpy import quat2dcm
from navpy import dcm2quat
import time as t

'''
____________STATE ESTIMATE VECTOR______________

        State                   unit        index
postion(xyz)                    m           1:3
velocity(xyz)                   m/s         4:6
orientation(quaternion)                     7:10
acc_bias_correction(xyz)        m/s²        11:13
acc_ortho_correction(3*3)                   14:22
gyr_ortho_correction(3*3)                   23:31
cam1_pos(xyz)                   m           32:34
cam1_ori(quaternion)                        35:38
cam1_k                          pixel        39
cam2_pos(xyz)                   m           40:42
cam2_ori(quaternion)                        43:46
cam2_k                          pixel        47



___________COMMANDS VECTOR_____________________

        State                   unit        index
gyroscopes(xyz)                 rad/s       1:3
accelerometers(xyz)             m/s²        4:6


___________SENSORS VECTOR_____________________

        State                   unit       index
beac1_cam1(uv)                  pixel       1:2
beac2_cam1(uv)                  pixel       3:4
beac1_cam2(uv)                  pixel       5:6
beac2_cam2(uv)                  pixel       7:8

matlab functions:
function next_X = transitionFunction(prev_X, U)

        %fonction de transition d'état (pour la prédiction à l'aide des
        %commandes ou capteurs proprioceptifs)
        
        dt = 1/100;

        [x,y,z,vx,vy,vz,qw,qx,qy,qz,abx,aby,abz] = deal(prev_X(1),prev_X(2),prev_X(3),prev_X(4),prev_X(5),prev_X(6),prev_X(7),prev_X(8),prev_X(9),prev_X(10),prev_X(11),prev_X(12),prev_X(13));
        [gx,gy,gz,ax,ay,az] = deal(U(1),U(2),U(3),(abx+U(4)),(aby+U(5)),(abz+U(6)));
        g=9.812;
        acc_o = reshape(prev_X(14:22),3,3);
       
        ac = [ax,ay,az] *acc_o;
        [ax,ay,az] = deal(ac(1),ac(2),ac(3));

        x_ = x+vx*dt;
	    y_ = y+vy*dt;
	    z_ = z+vz*dt;					% integration des vitesses linéaires

        [ae_x,ae_y,ae_z] = rotate(ax,ay,az,qw,qx,qy,qz);
        vx_ = vx+ae_x*dt;
	    vy_ = vy+ae_y*dt;
	    vz_ = vz+(ae_z+g)*dt;				% integration des accélération linéaires
        
        gyr_o = reshape(prev_X(23:31),3,3);
        gc = [gx,gy,gz] *gyr_o;
        [gx,gy,gz] = deal(gc(1),gc(2),gc(3));

        [dqw,dqx,dqy,dqz] = quatMul(qw,qx,qy,qz,0,gx,gy,gz);
        q_ = [qw,qx,qy,qz] + 0.5*dt*[dqw,dqx,dqy,dqz];
        [qw_,qx_,qy_,qz_] = quatNormalize(q_(1),q_(2),q_(3),q_(4));

        next_X = transpose([x_,y_,z_,vx_,vy_,vz_,qw_,qx_,qy_,qz_,abx,aby,abz,prev_X(14),prev_X(15),prev_X(16),prev_X(17),prev_X(18),prev_X(19),prev_X(20),prev_X(21),prev_X(22),prev_X(23),prev_X(24),prev_X(25),prev_X(26),prev_X(27),prev_X(28),prev_X(29),prev_X(30),prev_X(31),prev_X(32),prev_X(33),prev_X(34),prev_X(35),prev_X(36),prev_X(37),prev_X(38),prev_X(39),prev_X(40),prev_X(41),prev_X(42),prev_X(43),prev_X(44),prev_X(45),prev_X(46),prev_X(47)]);
end


function Z_predict = measurementFunction(X)
        %fonction pour prédire les donnés capteurs exteroceptifs
        %Z_predict = [X(1); X(2); X(3)];
        % perspective projection for led1
        %function to predict exteroceptive sensor data
        %Z_predict  : l1c1x, l1c1y, l2c1x, l2c1y, l1c2x, l1c2y, l2c2x, l2c2y
        [x,y,z,qw,qx,qy,qz] = deal(X(1),X(2),X(3),X(7),X(8),X(9),X(10));


        %sensors settings
        cam1_p = X(32:34);
        cam1_q = X(35:38);
        cam1_k = X(39);
        cam2_p = X(40:42);
        cam2_q = X(43:46);
        cam2_k = X(47);


        ir1_p = [0.1 0 0]; % position of led1
        ir2_p = [-0.1 0 0]; % position of led2

        [l1x,l1y,l1z] = rotate(ir1_p(1),ir1_p(2),ir1_p(3),qw,-qx,-qy,-qz);
        l1x= l1x+x;
        l1y= l1y+y;
        l1z= l1z+z;
        [l1x,l1y,l1z] = rotate(l1x-cam1_p(1),l1y-cam1_p(2),l1z-cam1_p(3),cam1_q(1),-cam1_q(2),-cam1_q(3),-cam1_q(4));
        l1c1 = [l1x,l1y]*cam1_k/l1z;
        [l2x,l2y,l2z] = rotate(ir2_p(1),ir2_p(2),ir2_p(3),qw,-qx,-qy,-qz);
        [l2x,l2y,l2z] = rotate(l2x+x-cam1_p(1),l2y+y-cam1_p(2),l2z+z-cam1_p(3),cam1_q(1),-cam1_q(2),-cam1_q(3),-cam1_q(4));
        l2c1 = [l2x,l2y]*cam1_k/(l2z);

        [l1x,l1y,l1z] = rotate(ir1_p(1),ir1_p(2),ir1_p(3),qw,-qx,-qy,-qz);
        [l1x,l1y,l1z] = rotate(l1x+x-cam2_p(1),l1y+y-cam2_p(2),l1z+z-cam2_p(3),cam2_q(1),-cam2_q(2),-cam2_q(3),-cam2_q(4));
        l1c2 = [l1x,l1y]*cam2_k/(l1z);
        [l2x,l2y,l2z] = rotate(ir2_p(1),ir2_p(2),ir2_p(3),qw,-qx,-qy,-qz);
        [l2x,l2y,l2z] = rotate(l2x+x-cam2_p(1),l2y+y-cam2_p(2),l2z+z-cam2_p(3),cam2_q(1),-cam2_q(2),-cam2_q(3),-cam2_q(4));
        l2c2 = [l2x,l2y]*cam2_k/(l2z);

        Z_predict = [l1c1(1); l1c1(2); l2c1(1); l2c1(2); l1c2(1); l1c2(2); l2c2(1); l2c2(2)];
    end
'''

#defaut parameters
dt_imu = 1/100
g = 9.812

#defaut mounting camera parameters
beac1_pos = np.array([0.1,0,0]).reshape(3,1)
beac2_pos = np.array([-0.1,0,0]).reshape(3,1)
cam1_mounting_position = np.array([-2,2,-2.8]).reshape(3,1)
cam1_mounting_orientation = np.array([-np.pi/4,np.pi/4,0]).reshape(3,1)
cam2_mounting_position = np.array([-2,-2,-2.8]).reshape(3,1)
cam2_mounting_orientation = np.array([np.pi/4,np.pi/4,0]).reshape(3,1)
cam_res = 480
cam_AoV = 120*np.pi/180
cam_noise = 1
cam_mounting_accur_pos = 0.2
cam_mounting_accur_ori = 0.1
cam_AoV_accur = 0.001

#defaut mpu6050 parameters
gyr_noise = 0.1*np.pi/180
gyr_gain_tolerance = 0.03
gyr_cross_tolerance = 0.02
acc_noise = 0.008*9.81
acc_xy_bias_tolerance = 0.06*9.81
acc_z_bias_tolerance = 0.08*9.81
acc_gain_tolerance = 0.03
acc_cross_tolerance = 0.03





def f(X,U):
        '''
        transition function
        '''
        xyz = X[0:3]
        vxyz = X[3:6]
        q0,qvec = X[6],X[7:10]
        abxyz = np.array(X[10:13]).reshape(3,1)
        gyro = np.array(U[0:3]).reshape(3,1)
        acc= np.array(U[3:6]).reshape(3,1)
        gyro_o = X[22:31].reshape(3,3)
        acc_o = X[13:22].reshape(3,3)
        
        #correction of gyro and acc
        gyro = gyro_o@gyro
        acc = acc_o@(acc+abxyz)
        
        # integration of linear velocities
        xyz_ = xyz + vxyz*dt_imu
        
        # integration of linear accelerations
        print("acc = ",acc)
        qdcm = quat2dcm(q0,qvec)
        ae = qdcm@acc
        vxyz_ = vxyz + (ae+np.array([0,0,g]))*dt_imu
        
        # integration of orientation
        # dq = quat.quaternion(0,gyro[0],gyro[1],gyro[2])*q
        # gyrdcm = -quat2dcm(0,gyro[0:3])
        # print("qdcm = ",qdcm)
        # print("gyrdcm = ",gyrdcm)
        # gyrdcm_qdcm = gyrdcm@qdcm
        # print("gyrdcm_qdcm = ",gyrdcm_qdcm)
        # dq0,dqvec = dcm2quat(gyrdcm_qdcm)
        # print("dq0 = ",dq0)
        # print("dqvec = ",dqvec)
        # dq = np.array([dq0,dqvec[0],dqvec[1],dqvec[2]])
        dq = quatMul(q0,qvec,0,gyro)
        print("dq = ",dq)
        q_ = [q0,qvec[0],qvec[1],qvec[2]] + 0.5*dt_imu*dq
        # q_ = q_.normalize()
        
        return np.concatenate((xyz_,vxyz_,q_,X[11:48]))

def quatMul(q10,q1vec,q20,q2vec):
        '''
        multiplication of two quaternions
        q1vec dim = 3*1
        q2vec dim = 3*1
        '''
        print("q10 = ",q10)
        print("q1vec.shape = ",q1vec.shape)
        print("q20 = ",q20)
        print("q2vec.shape = ",q2vec.shape)
        
        q0 = q10*q20 - np.dot(q1vec,np.transpose(q2vec))
        qvec = q10*q2vec + q20*q1vec + np.cross(q1vec,np.transpose(q2vec))
        return q0,qvec

def h(X):
        '''
        measurement function
        '''
        xyz = X[0:3]
        q = quat.quaternion(X[6:10])
        cam1_p = X[31:34]
        cam1_q = quat.quaternion(X[34],X[35],X[36],X[37])
        cam1_k = X[38]
        cam2_p = X[39:42]
        cam2_q = quat.quaternion(X[42],X[43],X[44],X[45])
        cam2_k = X[46]
        
        # projection of ir1
        l1 = q.rotate(beac1_pos)+xyz
        l1 = cam1_q.rotate(l1-cam1_p)
        l1 = l1[0:2]*cam1_k/l1[2]
        l1 = cam2_q.rotate(l1-cam2_p)
        l1 = l1[0:2]*cam2_k/l1[2]
        
        # projection of ir2
        l2 = q.rotate(beac2_pos)+xyz
        l2 = cam1_q.rotate(l2-cam1_p)
        l2 = l2[0:2]*cam1_k/l2[2]
        l2 = cam2_q.rotate(l2-cam2_p)
        l2 = l2[0:2]*cam2_k/l2[2]
        
        return np.concatenate((l1,l2))

'''
%basic specifications of mpu6050

gyr_noise = 0.1*pi/180 ; % rad/s
gyr_bias_tolerance = pi/180 ; %rad/s
gyr_gain_tolerance = 0.03;
gyr_cross_tolerance = 0.02;
gyr_bias_over_temp_tolerance = 0.24*pi/180; %rad/s/°C
gyr_gain_over_temp_tolerance = 0.00032*pi/180; % /°C
gyr_freq = 100; % Hz
gyr_res = 1/131*pi/180; %rad/s/lsb
gyr_axis_alignement = [1 0 0; 0 1 0; 0 0 1];
gyr_dynamic = 250*pi/180; %rad/s

acc_noise = 0.008*9.81 ; % m/s²
acc_xy_bias_tolerance = 0.06*9.81; % m/s²
acc_z_bias_tolerance = 0.08*9.81; % m/s²
acc_gain_tolerance = 0.03;
acc_cross_tolerance = 0.03;
acc_bias_over_temp_tolerance = 0.0015*9.81; % m/s²/°C
acc_gain_over_temp_tolerance = 0.00026; % /°C
acc_freq = 100; % Hz
acc_res = 1/16384*9.81; %m/s²/lsb  at 2g range
acc_axis_alignement = [1 0 0; 0 1 0; 0 0 1];
acc_dynamic = 2*9.81; % m/s²

%parameters of gps
gps_xy_noise = 0.1;
gps_z_noise  = 0.1;
gps_freq = 20;
gps_res = 0.001;

%beacons parameters
beac1_pos = [0.1 0 0]; %m
beac2_pos = [-0.1 0 0]; %m

% camera specification
cam_AoV = 120*pi/180; %rad
cam1_mounting_position = [-2 2 -2.8]; %xyz
cam1_mounting_orientation = [-pi/4 pi/4 0]; % rz ry rx
cam2_mounting_position = [-2 -2 -2.8]; %xyz

cam2_mounting_orientation = [pi/4 pi/4 0]; % rz ry rx
cam_res = 480;
cam_noise = 1; %pixels
cam_fps = 10;

cam_mounting_accur_pos = 0.2;%m
cam_mounting_accur_ori = 0.1;%rad
cam_AoV_accur = 0.001; %rad

% constant calculus
gyr_axis_alignement_inv = inv(gyr_axis_alignement);
gyr_bias = 2*(rand([1 3])-0.5)*gyr_bias_tolerance;
gyr_non_orthogonality = (rand([3 3])-0.5)*gyr_cross_tolerance; % cross axis deformation
gyr_non_orthogonality = gyr_non_orthogonality-diag(diag(gyr_non_orthogonality)); % clear diag
gyr_non_orthogonality = gyr_non_orthogonality+diag(1+2*(rand([1 3])-0.5)*gyr_gain_tolerance); %replace them by the gains
gyr_non_orthogonality = gyr_axis_alignement*gyr_non_orthogonality;
gyr_bias_over_temp = 2*(rand([1 3])-0.5)*gyr_bias_over_temp_tolerance;
gyr_gain_over_temp = diag(2*(rand([1 3])-0.5)*gyr_bias_over_temp_tolerance);

acc_axis_alignement_inv = inv(acc_axis_alignement);
acc_bias = [2*(rand-0.5)*acc_xy_bias_tolerance 2*(rand-0.5)*acc_xy_bias_tolerance 2*(rand-0.5)*acc_z_bias_tolerance]; 
acc_non_orthogonality = (rand([3 3])-0.5)*acc_cross_tolerance; % cross axis deformation
acc_non_orthogonality = acc_non_orthogonality-diag(diag(acc_non_orthogonality)); % clear diag
acc_non_orthogonality = acc_non_orthogonality+diag(1+2*(rand([1 3])-0.5)*acc_gain_tolerance); %replace them by the gains
acc_non_orthogonality = acc_axis_alignement*acc_non_orthogonality;
acc_bias_over_temp = 2*(rand([1 3])-0.5)*acc_bias_over_temp_tolerance;
acc_gain_over_temp = diag(2*(rand([1 3])-0.5)*acc_bias_over_temp_tolerance);

cam1_position = cam1_mounting_position+(rand([1 3])-0.5)*cam_mounting_accur_pos;
cam1_orientation = cam1_mounting_orientation + (rand([1 3])-0.5)*cam_mounting_accur_ori;
cam1_orientation_q = angle2quat(cam1_orientation(1),cam1_orientation(2),cam1_orientation(3));
cam1_k = cam_res/(2*tan((cam_AoV+(rand-0.5)*cam_AoV_accur)/2));

cam2_position = cam2_mounting_position+(rand([1 3])-0.5)*cam_mounting_accur_pos;
cam2_orientation = cam2_mounting_orientation + (rand([1 3])-0.5)*cam_mounting_accur_ori;
cam2_orientation_q = angle2quat(cam2_orientation(1),cam2_orientation(2),cam2_orientation(3));
cam2_k = cam_res/(2*tan((cam_AoV+(rand-0.5)*cam_AoV_accur)/2));

% covariance de l'état initial
ekf.Pn(1:10,1:10) = 0; 
ekf.Pn(11,11) = acc_xy_bias_tolerance^2;
ekf.Pn(12,12) = acc_xy_bias_tolerance^2;
ekf.Pn(13,13) = acc_z_bias_tolerance^2;
ekf.Pn(14:22,14:22) = diag(reshape((eye(3)*acc_gain_tolerance +ones(3,3)*acc_cross_tolerance-diag(diag(ones(3,3)*acc_cross_tolerance))).^2,9,[]));
ekf.Pn(23:31,23:31) = diag(reshape((eye(3)*gyr_gain_tolerance +ones(3,3)*gyr_cross_tolerance-diag(diag(ones(3,3)*gyr_cross_tolerance))).^2,9,[]));
ekf.Pn(32:34,32:34) = eye(3)*(cam_mounting_accur_pos^2);
ekf.Pn(35:38,35:38) = eye(4)*(0.2*cam_mounting_accur_ori)^2;
k_uncertainty = 0.5*(cam_res/(2*tan((cam_AoV+cam_AoV_accur)/2))-cam_res/(2*tan((cam_AoV-cam_AoV_accur)/2)));
ekf.Pn(39,39) = k_uncertainty^2;
ekf.Pn(40:42,40:42) = eye(3)*(cam_mounting_accur_pos^2);
ekf.Pn(43:46,43:46) = eye(4)*(0.2*cam_mounting_accur_ori)^2;
ekf.Pn(47,47) = k_uncertainty^2;
% initialisation des bruit de commande et de capteur
ekf.Q = 4*diag([gyr_rms(1),gyr_rms(2),gyr_rms(3),acc_rms(1),acc_rms(2),acc_rms(3)]);
%ekf.R = diag([f_sim*(0.1)^2, f_sim*(0.1)^2, f_sim*(0.1)^2]);
ekf.R = eye(8)*cam_noise^2;
'''   
#in python, list[0:n] is [0,n-1 

def init(ekf):
        ekf.Xn[0:13] = 0
        ekf.Xn[6] = 1
        ekf.Xn[13:22] = np.eye(3).reshape(9,1)
        ekf.Xn[22:31] = np.eye(3).reshape(9,1)
        ekf.Xn[31:34] = cam1_mounting_position
        q0,qvec=angle2quat(cam1_mounting_orientation[0],cam1_mounting_orientation[1],cam1_mounting_orientation[2])
        ekf.Xn[34] = q0
        ekf.Xn[35:38] = qvec.reshape(3,1)
        ekf.Xn[38] = cam_res/(2*np.tan(cam_AoV/2))
        ekf.Xn[39:42] = cam2_mounting_position
        q0,qvec=angle2quat(cam2_mounting_orientation[0],cam2_mounting_orientation[1],cam2_mounting_orientation[2])
        ekf.Xn[42] = q0
        ekf.Xn[43:46] = qvec.reshape(3,1)
        ekf.Xn[46] = cam_res/(2*np.tan(cam_AoV/2))
        
        ekf.Pn[0:10,0:10] = np.zeros((10,10))
        ekf.Pn[10,10] = acc_xy_bias_tolerance**2
        ekf.Pn[11,11] = acc_xy_bias_tolerance**2
        ekf.Pn[12,12] = acc_z_bias_tolerance**2
        ekf.Pn[13,13] = acc_noise**2
        ekf.Pn[13:22,13:22] = np.diag(np.reshape((np.eye(3)*acc_gain_tolerance +np.ones((3,3))*acc_cross_tolerance-np.diag(np.diag(np.ones((3,3))*acc_cross_tolerance)))**2,9))
        ekf.Pn[22:31,22:31] = np.diag(np.reshape((np.eye(3)*gyr_gain_tolerance +np.ones((3,3))*gyr_cross_tolerance-np.diag(np.diag(np.ones((3,3))*gyr_cross_tolerance)))**2,9))
        ekf.Pn[31:34,31:34] = np.eye(3)*(cam_mounting_accur_pos**2)
        ekf.Pn[34:38,34:38] = np.eye(4)*(0.2*cam_mounting_accur_ori)**2
        k_uncertainty = 0.5*(cam_res/(2*np.tan((cam_AoV+cam_AoV_accur)/2))-cam_res/(2*np.tan((cam_AoV-cam_AoV_accur)/2)))
        ekf.Pn[38,38] = k_uncertainty**2
        ekf.Pn[39:42,39:42] = np.eye(3)*(cam_mounting_accur_pos**2)
        ekf.Pn[42:46,42:46] = np.eye(4)*(0.2*cam_mounting_accur_ori)**2
        ekf.Pn[46,46] = k_uncertainty**2       

        ekf.Q = np.diag([gyr_noise,gyr_noise,gyr_noise,acc_noise,acc_noise,acc_noise])
        ekf.R = np.eye(8)*cam_noise**2

ekf = kf.myEKF(f,h,47,8,6)

time = None
gyr = None
acc = None
new_proprio = False
new_measure = False
beac1_cam1 = None
beac2_cam1 = None
beac1_cam2 = None
beac2_cam2 = None

last_time = 0
def predict():
        '''
        predict the state
        '''
        global dt_imu, last_time, new_proprio, time, gyr, acc
        dt_imu = (time-last_time)/1000.0
        last_time = time
        print("dt_imu = ",dt_imu)
        
        if (dt_imu>0.1):
                new_proprio = False
                return
                
        ekf.Un = np.concatenate((gyr,acc))
        ekf.predict()
        new_proprio = False



def update(beac1_cam1,beac2_cam1,beac1_cam2,beac2_cam2):
        '''
        update the state
        '''
        global new_measure
        ekf.Zn = np.concatenate((beac1_cam1,beac2_cam1,beac1_cam2,beac2_cam2))
        ekf.update()
        new_measure = False


def calibrationTask():
        init(ekf)
        while True:
                if new_proprio:
                        predict()
                        print(ekf.Xn[6:10])
                if new_measure:
                        update(beac1_cam1,beac2_cam1,beac1_cam2,beac2_cam2)
                t.sleep(0.005)
                                 
def main():
        threading.Thread(target=calibrationTask).start()
