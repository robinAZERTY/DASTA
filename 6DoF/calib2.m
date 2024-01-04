clearvars;
clc;
run sim_cfg.m
out = sim('staticCalibrationDataGenerator.slx',T_sim_cal_static);

[~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,gyr_xx,gyr_yy,gyr_zz,acc_xx,acc_yy,acc_zz,~,~,~,~] = simoutFormat(out.static);


gyr_avr = mean([gyr_xx gyr_yy gyr_zz]);
gyr_bias_correction = -gyr_avr;
gyr_rms = mean([gyr_xx.^2 gyr_yy.^2 gyr_zz.^2])-gyr_avr.^2;
gyr_bias_correction_var = gyr_rms/length(gyr_xx);

avr_acc = mean([acc_xx acc_yy acc_zz]);
acc_rms = mean([acc_xx.^2 acc_yy.^2 acc_zz.^2])-avr_acc.^2;

%second phase of calibration : ekf
out = sim('dynamicCalibrationDataGenerator.slx',T_sim_cal_dynamic);
[t,x,y,z,vx,vy,vz,ax,ay,az,qw,qx,qy,qz,wxx,wyy,wzz,axx,ayy,azz,gyr_xx,gyr_yy,gyr_zz,acc_xx,acc_yy,acc_zz,temperature, ~, ~, ~] = simoutFormat(out.random_mov);

%{
____________STATE ESTIMATE VECTOR______________

        State                   unit        index
postion(xyz)                    m           1:3
velocity(xyz)                   m/s         4:6
orientation(quaternion)                     7:10
acc_bias_correction(xyz)        m/s²        11:13


___________COMMANDS VECTOR_____________________

        State                   unit        index
gyroscopes(xyz)                 rad/s       1:3
accelerometers(xyz)             m/s²        4:6

%}

 % Fonction de transition d'état
f = @(prev_X, U) transitionFunction(prev_X, U);

% Fonction de mesure
h = @(X) measurementFunction(X);

% Créez une instance de la classe EKF avec les fonctions définies
ekf = myEKF(f, h, 13, 3, 6, [], [], []); % 34 dimensions d'état, 3 dimensions de mesure, 6 dimensions de commandes

% initialisation des bruit de commande et de capteur
ekf.Q = 4*diag([gyr_rms(1),gyr_rms(2),gyr_rms(3),acc_rms(1),acc_rms(2),acc_rms(3)]);
ekf.R = diag([f_sim*(0.1)^2, f_sim*(0.1)^2, f_sim*(0.1)^2]);

% état initial
% position et vitesse nulles
% orientation à plat et cap de 0
% biais nuls et gains de 1
ekf.Xn(:) = 0; 
ekf.Xn(7) = 1;
ekf.Xn(14:16) = 1;

% covariance de l'état initial
ekf.Pn(1:10,1:10) = 0; 
ekf.Pn(11,11) = acc_xy_bias_tolerance^2;
ekf.Pn(12,12) = acc_xy_bias_tolerance^2;
ekf.Pn(13,13) = acc_z_bias_tolerance^2;


X_est = zeros([length(t),13]);
P_est = zeros([length(t),13,13]);

for i = 1:length(t)
     % Prédiction de l'état
    ekf.Un = [gyr_xx(i)+gyr_bias_correction(1); gyr_yy(i)+gyr_bias_correction(2); gyr_zz(i)++gyr_bias_correction(3); acc_xx(i); acc_yy(i); acc_zz(i)];
    ekf = ekf.predict();
    ekf.Zn = [x(i);y(i);z(i)];% le système reste proche de l'origine
    ekf.Zn = ekf.Zn + randn(3,1)*0.1;
    ekf.Pn(11,11) = ekf.Pn(11,11)+acc_xy_bias_tolerance^2 * 1e-6;
    ekf.Pn(12,12) = ekf.Pn(12,12)+acc_xy_bias_tolerance^2 * 1e-6;
    ekf.Pn(13,13) = ekf.Pn(13,13)+acc_z_bias_tolerance^2  * 1e-6;
    ekf = ekf.update();
    X_est(i,:) = ekf.Xn;
    P_est(i,:,:) = ekf.Pn;
end

figure('Name', 'P');
image(abs(sqrt(ekf.Pn)),'CDataMapping','scaled');
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

subplot(2, 1, 1);
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

function next_X = transitionFunction(prev_X, U)

        %fonction de transition d'état (pour la prédiction à l'aide des
        %commandes ou capteurs proprioceptifs)
        
        dt = 1/100;
        g=9.81;

        [x,y,z,vx,vy,vz,qw,qx,qy,qz,abx,aby,abz] = deal(prev_X(1),prev_X(2),prev_X(3),prev_X(4),prev_X(5),prev_X(6),prev_X(7),prev_X(8),prev_X(9),prev_X(10),prev_X(11),prev_X(12),prev_X(13));
        [gx,gy,gz,ax,ay,az] = deal(U(1),U(2),U(3),(abx+U(4)),(aby+U(5)),(abz+U(6)));

        x_ = x+vx*dt;
	    y_ = y+vy*dt;
	    z_ = z+vz*dt;					% integration des vitesses linéaires

        [ae_x,ae_y,ae_z] = rotate(ax,ay,az,qw,-qx,-qy,-qz);
        vx_ = vx+ae_x*dt;
	    vy_ = vy+ae_y*dt;
	    vz_ = vz+(ae_z-g)*dt;				% integration des accélération linéaires
        
        
        [dqw,dqx,dqy,dqz] = quatMul(qw,qx,qy,qz,0,gx,gy,gz);
        q_ = [qw,qx,qy,qz] + 0.5*dt*[dqw,dqx,dqy,dqz];
        [qw_,qx_,qy_,qz_] = quatNormalize(q_(1),q_(2),q_(3),q_(4));

        next_X = [x_;y_;z_;vx_;vy_;vz_;qw_;qx_;qy_;qz_;abx;aby;abz];
end

function Z_predict = measurementFunction(X)
        %fonction pour prédire les donnés capteurs exteroceptifs
        Z_predict = [X(1); X(2); X(3)];
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
