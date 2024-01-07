clearvars;
clc;
run sim_cfg.m
out = sim('staticCalibrationDataGenerator.slx',T_sim_cal_static);

[t,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,gyr_xx,gyr_yy,gyr_zz,acc_xx,acc_yy,acc_zz,~,~,~,~,~,~,~,~,~,~,~] = simoutFormat(out.static);

n_static_cal = round(t(end)*gyr_freq);
gyr_avr = mean([gyr_xx gyr_yy gyr_zz]);
gyr_bias_correction = -gyr_avr;
gyr_rms = mean([gyr_xx.^2 gyr_yy.^2 gyr_zz.^2])-gyr_avr.^2;
gyr_bias_correction_var = gyr_rms/length(gyr_xx);

avr_acc = mean([acc_xx acc_yy acc_zz]);
acc_rms = mean([acc_xx.^2 acc_yy.^2 acc_zz.^2])-avr_acc.^2;

%second phase of calibration : ekf
out = sim('dynamicCalibrationDataGenerator.slx',T_sim_cal_dynamic);
[t,x,y,z,vx,vy,vz,ax,ay,az,qw,qx,qy,qz,wxx,wyy,wzz,axx,ayy,azz,gyr_xx,gyr_yy,gyr_zz,acc_xx,acc_yy,acc_zz,temperature, gpsx, gpsy, gpsz, l1c1x, l1c1y, l2c1x, l2c1y, l1c2x, l1c2y, l2c2x, l2c2y] = simoutFormat(out.random_mov);

%{
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

%}

 % Fonction de transition d'état
f = @(prev_X, U) transitionFunction(prev_X, U);

% Fonction de mesure
h = @(X) measurementFunction(X);

% Créez une instance de la classe EKF avec les fonctions définies
ekf = myEKF(f, h, 47, 8, 6, [], [], []);

% initialisation des bruit de commande et de capteur
ekf.Q = 4*diag([gyr_rms(1),gyr_rms(2),gyr_rms(3),acc_rms(1),acc_rms(2),acc_rms(3)]);
%ekf.R = diag([f_sim*(0.1)^2, f_sim*(0.1)^2, f_sim*(0.1)^2]);
ekf.R = eye(8)*cam_noise^2;

% état initial
% position et vitesse nulles
% orientation à plat et cap de 0
% biais nuls et gains de 1
ekf.Xn(:) = 0; 
ekf.Xn(7) = 1;
ekf.Xn(14:22) = reshape(eye(3,3),9,[]);
ekf.Xn(23:31) = reshape(eye(3,3),9,[]);
ekf.Xn(32:34) = cam1_mounting_position;
ekf.Xn(35:38) = angle2quat(cam1_mounting_orientation(1),cam1_mounting_orientation(2),cam1_mounting_orientation(3));
ekf.Xn(39) = cam_res/(2*tan(cam_AoV/2));
ekf.Xn(40:42) = cam2_mounting_position;
ekf.Xn(43:46) = angle2quat(cam2_mounting_orientation(1),cam2_mounting_orientation(2),cam2_mounting_orientation(3));
ekf.Xn(47) = cam_res/(2*tan(cam_AoV/2));


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

X_est = zeros([length(t),47]);
P_est = zeros([length(t),47,47]);


last_cam_t =-1;
for i = 1:length(t)
    %pause(1)
    clc;
    current_t=t(i)

     % Prédiction de l'état
    ekf.Un = [gyr_xx(i)+gyr_bias_correction(1); gyr_yy(i)+gyr_bias_correction(2); gyr_zz(i)+gyr_bias_correction(3); acc_xx(i); acc_yy(i); acc_zz(i)];
    ekf = ekf.predict();
  
    if current_t-last_cam_t>=1/cam_fps && abs(current_t-90)>1
        last_cam_t = current_t;
        ekf.Zn = [l1c1x(i); l1c1y(i); l2c1x(i); l2c1y(i); l1c2x(i); l1c2y(i); l2c2x(i); l2c2y(i)];
        ekf = ekf.update();
    end
    X_est(i,:) = ekf.Xn;
    P_est(i,:,:) = ekf.Pn;

end

figure('Name', 'P');
subplot(1,2,1);
image(log(abs(sqrt(ekf.Pn))),'CDataMapping','scaled');
subtitle("log scale of P");
colorbar;
subplot(1,2,2);
image(abs(sqrt(ekf.Pn)),'CDataMapping','scaled');
subtitle("abs sqrt of P");
colorbar;


% Affichage de la vue en 3D de la Pose (trajectoire, vitesse et
% orientation)
figure("Name","Pose réelle vs estimation");
subplot(1,2,1);
subtitle('Trajectoire Estimée vs Trajectoire Réelle avec Mesures GPS');
plot3(x, y, z, 'g-', 'LineWidth', 2); % Trajectoire réelle en vert
hold on;
plot3(X_est(:,1), X_est(:,2), X_est(:,3), 'b-', 'LineWidth', 2); % Trajectoire estimée en bleu
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Trajectoire Réelle', 'Trajectoire Estimée');
grid on;
hold off;
axis equal;

subplot(2,2,2);
hold on;
plot(t, X_est(:,4), 'r--', 'LineWidth', 1.5); % v_x estimé
plot(t, X_est(:,5), 'g--', 'LineWidth', 1.5); % v_y estimé
plot(t, X_est(:,6), 'b--', 'LineWidth', 1.5); % v_z estimé
plot(t, vx, 'r-', 'LineWidth', 1.5); % v_x réel en pointillé
plot(t, vy, 'g-', 'LineWidth', 1.5); % v_y réel en pointillé
plot(t, vz, 'b-', 'LineWidth', 1.5); % v_z réel en pointillé

% Incertitudes en pointillé
plot(t, X_est(:,4) + sqrt(squeeze(P_est(:,4,4))), 'r--', 'LineWidth', 0.5); % v_x + sigma
plot(t, X_est(:,4) - sqrt(squeeze(P_est(:,4,4))), 'r--', 'LineWidth', 0.5); % v_x - sigma
plot(t, X_est(:,5) + sqrt(squeeze(P_est(:,5,5))), 'g--', 'LineWidth', 0.5); % v_y + sigma
plot(t, X_est(:,5) - sqrt(squeeze(P_est(:,5,5))), 'g--', 'LineWidth', 0.5); % v_y - sigma
plot(t, X_est(:,6) + sqrt(squeeze(P_est(:,6,6))), 'b--', 'LineWidth', 0.5); % v_z + sigma
plot(t, X_est(:,6) - sqrt(squeeze(P_est(:,6,6))), 'b--', 'LineWidth', 0.5); % v_z - sigma


title('Vitesses Estimées vs Réelles');
legend('v_x estimé', 'v_y estimé', 'v_z estimé', 'v_x réel', 'v_y réel', 'v_z réel');
xlabel('Temps');
ylabel('Vitesse');
grid on;
hold off;

% Quaternion d'orientation
subplot(2,2,4);
hold on;
plot(t, qw, 'r-', 'LineWidth', 1.5); % qw
plot(t, qx, 'g-', 'LineWidth', 1.5); % qx
plot(t, qy, 'b-', 'LineWidth', 1.5); % qy
plot(t, qz, 'k-', 'LineWidth', 1.5); % qz

plot(t, X_est(:,7), 'r--', 'LineWidth', 1.5); % qw estimé
plot(t, X_est(:,8), 'g--', 'LineWidth', 1.5); % qx estimé
plot(t, X_est(:,9), 'b--', 'LineWidth', 1.5); % qy estimé
plot(t, X_est(:,10), 'k--', 'LineWidth', 1.5); % qz estimé

% Incertitudes en pointillé
plot(t, X_est(:,7) + sqrt(squeeze(P_est(:,7,7))), 'r--', 'LineWidth', 0.5); % qw + sigma
plot(t, X_est(:,7) - sqrt(squeeze(P_est(:,7,7))), 'r--', 'LineWidth', 0.5); % qw - sigma
plot(t, X_est(:,8) + sqrt(squeeze(P_est(:,8,8))), 'g--', 'LineWidth', 0.5); % qx + sigma
plot(t, X_est(:,8) - sqrt(squeeze(P_est(:,8,8))), 'g--', 'LineWidth', 0.5); % qx - sigma
plot(t, X_est(:,9) + sqrt(squeeze(P_est(:,9,9))), 'b--', 'LineWidth', 0.5); % qy + sigma
plot(t, X_est(:,9) - sqrt(squeeze(P_est(:,9,9))), 'b--', 'LineWidth', 0.5); % qy - sigma
plot(t, X_est(:,10) + sqrt(squeeze(P_est(:,10,10))), 'k--', 'LineWidth', 0.5); % qz + sigma
plot(t, X_est(:,10) - sqrt(squeeze(P_est(:,10,10))), 'k--', 'LineWidth', 0.5); % qz - sigma

title('Quaternion d''Orientation Estimé vs réel avec Incertitudes');
legend('qw', 'qx', 'qy', 'qz', 'qw estimé', 'qx estimé', 'qy estimé', 'qz estimé');
xlabel('Temps');
ylabel('Valeur');
grid on;
hold off;

% Biais de l'accéléromètre
figure('Name',"accéléromètres");

subplot(4, 1, 1);
hold on;
plot(t,acc_bias(1)*ones([1 length(t)]),'r-');
plot(t,acc_bias(2)*ones([1 length(t)]), 'g-');
plot(t,acc_bias(3)*ones([1 length(t)]), 'b-');

plot(t, -X_est(:,11)+sqrt(squeeze(P_est(:,11,11))), 'r--', 'LineWidth', 0.5); % qw + sigma
plot(t, -X_est(:,11)-sqrt(squeeze(P_est(:,11,11))), 'r--', 'LineWidth', 0.5); % qw + sigma

plot(t, -X_est(:,12)+sqrt(squeeze(P_est(:,12,12))), 'g--', 'LineWidth', 0.5); % qw + sigma
plot(t, -X_est(:,12)-sqrt(squeeze(P_est(:,12,12))), 'g--', 'LineWidth', 0.5); % qw - sigma

plot(t, -X_est(:,13)+sqrt(squeeze(P_est(:,13,13))), 'b--', 'LineWidth', 0.5); % qw + sigma
plot(t, -X_est(:,13)-sqrt(squeeze(P_est(:,13,13))), 'b--', 'LineWidth', 0.5); % qw - sigma


title('Biais de l''Accéléromètre');
legend('t_x','t_y','t_z','X', 'Y', 'Z');
grid on;


% Matrice de non-orthogonalité de l'accéléromètre
acc_non_orthogonality_inv = inv(acc_non_orthogonality);
for i = 1:3
    for j=1:3
        subplot(4, 3, 3+(i-1)*3+j);
        hold on;
        plot(t, acc_non_orthogonality_inv(i, j) * ones(size(t)), '--', 'LineWidth', 1); % Ligne pour la valeur vraie
        plot(t, X_est(:,13 + (i-1)*3+j), 'b--', 'LineWidth', 1); % qw + sigma
        plot(t, X_est(:,13 + (i-1)*3+j)+sqrt(squeeze(P_est(:,13 + (i-1)*3+j,13 + (i-1)*3+j))), 'b--', 'LineWidth', 0.5); % qw + sigma
        plot(t, X_est(:,13 + (i-1)*3+j)-sqrt(squeeze(P_est(:,13 + (i-1)*3+j,13 + (i-1)*3+j))), 'b--', 'LineWidth', 0.5); % qw + sigma
        title(['ao' num2str(i) num2str(j)]);
        grid on;
    end
end

% Biais de l'accéléromètre
figure('Name',"gyroscopes");


% Matrice de non-orthogonalité de l'accéléromètre
gyr_non_orthogonality_inv = inv(gyr_non_orthogonality);
for i = 1:3
    for j=1:3
        subplot(3, 3, (i-1)*3+j);
        hold on;
        plot(t, gyr_non_orthogonality_inv(i, j) * ones(size(t)), '--', 'LineWidth', 1); % Ligne pour la valeur vraie
        plot(t, X_est(:,22 + (i-1)*3+j), 'b--', 'LineWidth', 1); % qw + sigma
        plot(t, X_est(:,22 + (i-1)*3+j)+sqrt(squeeze(P_est(:,22 + (i-1)*3+j,22 + (i-1)*3+j))), 'b--', 'LineWidth', 0.5); % qw + sigma
        plot(t, X_est(:,22 + (i-1)*3+j)-sqrt(squeeze(P_est(:,22 + (i-1)*3+j,22 + (i-1)*3+j))), 'b--', 'LineWidth', 0.5); % qw + sigma
        title(['go' num2str(i) num2str(j)]);
        grid on;
    end
end


%{
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
%}

tp = theaterPlot('XLimit',[-3 3],'YLimit',[-3 3],'ZLimit',[-3.5 1.5]);
opcam = orientationPlotter(tp,'DisplayName','Cam positions estimate',...
    'LocalAxesLength',0.5,'Marker','*');

opcam_true = orientationPlotter(tp,'DisplayName','true Cam positions',...
    'LocalAxesLength',0.2,'Marker','.');

opquad = orientationPlotter(tp,'DisplayName','quadricopter position estimate',...
    'LocalAxesLength',1);

opquad_true = orientationPlotter(tp,'DisplayName','true quadricopter position',...
    'LocalAxesLength',0.4,'Marker','.');


plotOrientation(opcam_true,[quaternion(cam1_orientation_q);quaternion(cam2_orientation_q)],[cam1_position; cam2_position]);

for i = 1:size(X_est,1)
    clc;
    ct = t(i)
    o=quaternion(X_est(i,7:10));
    p = X_est(i,1:3);
    oc1=quaternion(X_est(i,35:38));
    pc1 = X_est(i,32:34);
    oc2=quaternion(X_est(i,43:46));
    pc2 = X_est(i,40:42);
    Os = [oc1;oc2];
    Ps = [pc1;pc2];
    plotOrientation(opquad,o,p);
    plotOrientation(opquad_true,quaternion([qw(i) qx(i) qy(i) qz(i)]),[x(i) y(i) z(i)]);
    plotOrientation(opcam,Os,Ps);
    drawnow
end


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

function [x2,y2,z2] = rotate(x1,y1,z1,qw,qx,qy,qz)
    [q3w,q3x,q3y,q3z] = quatMul(qw,qx,qy,qz,0,x1,y1,z1);
    [~,x2,y2,z2] = quatMul(q3w,q3x,q3y,q3z,qw,-qx,-qy,-qz);
end

function [q3w,q3x,q3y,q3z] = quatMul(q1w,q1x,q1y,q1z,q2w,q2x,q2y,q2z)
    q3w = q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z;
    q3x = q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y;
    q3y = q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x;
    q3z = q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w;
end


function [q2w,q2x,q2y,q2z] = quatNormalize(q1w,q1x,q1y,q1z)
    norm = sqrt(q1w^2+q1x^2+q1y^2+q1z^2);
    q2w = q1w/norm;
    q2x = q1x/norm;
    q2y = q1y/norm;
    q2z = q1z/norm;
end
