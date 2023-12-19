% Parameters for quadcopter_package_delivery
% Copyright 2021-2023 The MathWorks, Inc.

% Size of the ground
planex = 12.5;           % m
planey = 8.5;            % m
planedepth = 0.2;        % m, distance from plane to the reference frame

% Battery Capacity
battery_capacity = 7.6*3;

%% Material Property
% Assuming the arm of the drone is manufractured by 3D Printing, the ideal
% material is PLA, safe, light and cheap, the only concern is its thermal
% property
rho_pla   = 1.25;            % g/cm^3 

% Measured drone mass
drone_mass = 1.2726;
%% package ground contact properties
pkgGrndStiff  = 1000;
pkgGrndDamp   = 300;
pkgGrndTransW = 1e-3;


%% Package parameters
pkgSize = [1 1 1]*0.14; % m
pkgDensity = 1/(pkgSize(1)*pkgSize(2)*pkgSize(3)); % kg/m^3

%% Propeller parameters
propeller.diameter = 0.254; % m
propeller.Kthrust  = 0.1072; 
propeller.Kdrag    = 0.01;

air_rho            = 1.225;  % kg/m^3
air_temperature    = 273+25; % degK
wind_speed         = 0;      % Wind speed (m/s)

%% Leg parameters
drone_leg.Extr_Data = flipud([...
    0     0;
    0.5   0;
    1    -1;
    0.98 -1;
    0.5  -0.02;
   -0.5  -0.02;
   -0.98 -1;
   -1    -1;
   -0.5   0].*[1 1]*0.15);

drone_leg.width = 0.01;

%% Motor parameters
qc_motor.max_torque = 0.8;  % N*m
qc_motor.max_power  = 160;  % W
qc_motor.time_const = 0.02; % sec
qc_motor.efficiency = 25/30*100; % 0-100
qc_motor.efficiency_spd = 5000; % rpm
qc_motor.efficiency_trq = 0.05; % N*m
qc_motor.rotor_damping  = 1e-7; % N*m/(rad/s)

qc_max_power = qc_motor.max_power;

%% Controller parameters

filtM_position = 0.005;
kp_position    = 8;
ki_position    = 0.04;
kd_position    = 3.2;
filtD_position = 100;
pos2attitude   = 2.4;

filtM_attitude = 0.01;
kp_attitude    = 128.505;
ki_attitude    = 5.9203;
kd_attitude    = 1.5*78.2000*2;
filtD_attitude = 0.1*1000;
limit_attitude = 800;

filtM_yaw      = 0.01;
kp_yaw         = 25.7010*4*2;
ki_yaw         = 5.9203*0.01;
kd_yaw         = 1.5*78.2000*0.01;
filtD_yaw      = 0.1*100;
limit_yaw      = 40;

filtM_altitude = 0.05;
kp_altitude    = 0.27;
ki_altitude    = 0.07;
kd_altitude    = 1.5*0.35;
filtD_altitude = 0.1*10000;
limit_altitude = 10;

kp_motor       = 0.00375;
ki_motor       = 4.50000e-4;
kd_motor       = 0;
filtD_motor    = 10000;
filtSpd_motor    = 0.001;
limit_motor    = 0.25;

%% Drag coefficients
qd_drag.Cd_X = 0.35;
qd_drag.Cd_Y = 0.35;
qd_drag.Cd_Z = 0.6;
qd_drag.Roll = 0.2;
qd_drag.Pitch = 0.2;
qd_drag.Yaw = 0.2;
qd_area.YZ = 0.0875;
qd_area.XZ = 0.0900;
qd_area.XY = 0.2560;
qd_area.Roll = qd_area.XY*2;
qd_area.Pitch = qd_area.XY*2;
qd_area.Yaw = qd_area.XY;

gravity = 9.80665;

%basic specifications of mpu6050

gyr_noise = 0.1*pi/180 ; % rad/s
gyr_bias_tolerance = 0.0001*5*pi/180 ; %rad/s
gyr_gain_tolerance = 0.0001*0.03;
gyr_cross_tolerance = 0.0001*0.02;
gyr_freq = 100; % Hz
gyr_res = 1/131*pi/180; %rad/s/lsb
gyr_axis_alignement = [1 0 0; 0 1 0; 0 0 1];
gyr_dynamic = 250*pi/180; %rad/s

acc_noise = 0.008*9.81 ; % m/s²
acc_xy_bias_tolerance = 0.0001*0.06*9.81; % m/s² , 0.0001* is for assuming acc is already calibrated
acc_z_bias_tolerance = 0.0001*0.08*9.81; % m/s²
acc_gain_tolerance = 0.0001*0.03;
acc_cross_tolerance = 0.0001*0.03;
acc_freq = 100; % Hz
acc_res = 1/16384*9.81; %m/s²/lsb  at 2g range
acc_axis_alignement = [1 0 0; 0 1 0; 0 0 1];
acc_dynamic = 2*9.81; % m/s²

%ir led
ir1_pos = [0.1 0 0]; %m
ir2_pos = [-0.1 0 0]; %m

% camera specification
cam_focale = 0.005; %m
cam_AoV = 120*pi/180; %rad
cam_position = [0 0 2.8]; %xyz
cam_orientation = [0 -pi 0]; % yaw pitch roll
cam_res = 480;
cam_noise = 1; %pixels
cam_fps = 10;

% constant calculus
gyr_axis_alignement_inv = inv(gyr_axis_alignement);
gyr_bias = 2*(rand([1 3])-0.5)*gyr_bias_tolerance;
gyr_non_orthogonality = (rand([3 3])-0.5)*gyr_cross_tolerance; % cross axis deformation
gyr_non_orthogonality = gyr_non_orthogonality-diag(gyr_non_orthogonality); % clear diag
gyr_non_orthogonality = gyr_non_orthogonality+diag(1+2*(rand([1 3])-0.5)*gyr_gain_tolerance); %replace them by the gains
gyr_non_orthogonality = gyr_axis_alignement*gyr_non_orthogonality;


acc_axis_alignement_inv = inv(acc_axis_alignement);
acc_bias = [2*(rand-0.5)*acc_xy_bias_tolerance 2*(rand-0.5)*acc_xy_bias_tolerance 2*(rand-0.5)*acc_z_bias_tolerance]; 
acc_non_orthogonality = (rand([3 3])-0.5)*acc_cross_tolerance; % cross axis deformation
acc_non_orthogonality = acc_non_orthogonality-diag(acc_non_orthogonality); % clear diag
acc_non_orthogonality = acc_non_orthogonality+diag(1+2*(rand([1 3])-0.5)*acc_gain_tolerance); %replace them by the gains
acc_non_orthogonality = acc_axis_alignement*acc_non_orthogonality;


cam_q = eul2quat(cam_orientation);
cam_l = cam_focale*tan(cam_AoV/2);

