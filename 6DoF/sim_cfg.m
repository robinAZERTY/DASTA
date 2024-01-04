%basic specifications of mpu6050

gyr_noise = 0.1*pi/180 ; % rad/s
gyr_bias_tolerance = 5*pi/180 ; %rad/s
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
beaconA_mounting_position = [-3 3 3];
beaconB_mounting_position = [3 3 3];
beaconC_mounting_position = [0 -3 3];
beacon_mounting_precision_tolerance = 0.3;
beacon_noise = 0.1;
beacon_bias_tolerance = 1;
beacon_gain_tolerance = 0.05;



% simulation param
f_sim = 100;
temp_dynamic = 1; % max-min temp
gravity = 9.81+rand()*0.005; % m/s²
T_sim_cal_static = 5 ;
T_sim_cal_dynamic = 30;
random_move_power = 3 ;


% constant calculus
gyr_axis_alignement_inv = inv(gyr_axis_alignement);
gyr_bias = 2*(rand([1 3])-0.5)*gyr_bias_tolerance;
gyr_non_orthogonality = (rand([3 3])-0.5)*gyr_cross_tolerance; % cross axis deformation
gyr_non_orthogonality = gyr_non_orthogonality-diag(gyr_non_orthogonality); % clear diag
gyr_non_orthogonality = gyr_non_orthogonality+diag(1+2*(rand([1 3])-0.5)*gyr_gain_tolerance); %replace them by the gains
gyr_non_orthogonality = gyr_axis_alignement*gyr_non_orthogonality;
%gyr_non_orthogonality = transpose(gyr_non_orthogonality);
gyr_bias_over_temp = 2*(rand([1 3])-0.5)*gyr_bias_over_temp_tolerance;
gyr_gain_over_temp = diag(2*(rand([1 3])-0.5)*gyr_bias_over_temp_tolerance);

acc_axis_alignement_inv = inv(acc_axis_alignement);
acc_bias = [2*(rand-0.5)*acc_xy_bias_tolerance 2*(rand-0.5)*acc_xy_bias_tolerance 2*(rand-0.5)*acc_z_bias_tolerance]; 
acc_non_orthogonality = (rand([3 3])-0.5)*acc_cross_tolerance; % cross axis deformation
acc_non_orthogonality = acc_non_orthogonality-diag(acc_non_orthogonality); % clear diag
acc_non_orthogonality = acc_non_orthogonality+diag(1+2*(rand([1 3])-0.5)*acc_gain_tolerance); %replace them by the gains
acc_non_orthogonality = acc_axis_alignement*acc_non_orthogonality;
%acc_non_orthogonality = transpose(acc_non_orthogonality);
acc_bias_over_temp = 2*(rand([1 3])-0.5)*acc_bias_over_temp_tolerance;
acc_gain_over_temp = diag(2*(rand([1 3])-0.5)*acc_bias_over_temp_tolerance);


