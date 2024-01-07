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



% simulation param
f_sim = acc_freq;
temp_dynamic = 1; % max-min temp
gravity = -9.812+(rand-0.5)*2*0.001; % m/s²
T_sim_cal_static = 5 ;
T_sim_cal_dynamic = 100;
random_move_power = 0.3 ;
random_move_delta_t = 0.2;


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
