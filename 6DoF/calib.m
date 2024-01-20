clearvars;
clc;
run sim_cfg.m
out = sim('staticCalibrationDataGenerator.slx',T_sim_cal_static);

[t,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,~,gyr_xx,gyr_yy,gyr_zz,acc_xx,acc_yy,acc_zz,~,~,~,~, l1c1x, l1c1y, l2c1x, l2c1y, l1c2x, l1c2y, l2c2x, l2c2y] = simoutFormat(out.static);


n_static_cal = round(t(end)*gyr_freq);
gyr_avr = mean([gyr_xx gyr_yy gyr_zz]);
gyr_bias_correction = -gyr_avr;
gyr_rms = mean([gyr_xx.^2 gyr_yy.^2 gyr_zz.^2])-gyr_avr.^2;
gyr_bias_correction_var = gyr_rms/length(gyr_xx);

avr_acc = mean([acc_xx acc_yy acc_zz]);
acc_rms = mean([acc_xx.^2 acc_yy.^2 acc_zz.^2])-avr_acc.^2;


%second phase of calibration : ekf
out = sim('dynamicCalibrationDataGenerator.slx',T_sim_cal_dynamic);
[t,x,y,z,vx,vy,vz,ax,ay,az,qw,qx,qy,qz,wxx,wyy,wzz,axx,ayy,azz,gyr_xx,gyr_yy,gyr_zz,acc_xx,acc_yy,acc_zz,temperature, gps_x, gps_y, gps_z, l1c1x, l1c1y, l2c1x, l2c1y, l1c2x, l1c2y, l2c2x, l2c2y] = simoutFormat(out.random_mov);

%X_est = [P_xyz q V_xyz acc_bias_correction acc_non_orthogonality_correction gyr_bias_correction gyr_non_orthogonality_correction,g]
%X_est = [x y z qw qx qy qz vx vy vz abx aby abz ao11 ao12 ao13 ao21 ao22
%ao23 ao31 ao32 ao33 gbx gby gbz go11 go12 go13 go21 go22 go23 go31 go32
%go33 g] -> length = 35
%U = [gx gy gz ax ay az]


 % Fonction de transition d'état
f = @(prev_X, U) transitionFunction(prev_X, U);

% Fonction de mesure
h = @(X) measurementFunction(X);

% Créez une instance de la classe EKF avec les fonctions définies
ekf = myEKF(f, h, 35, 3, 6, [], [], []); % 34 dimensions d'état, 3 dimensions de mesure, 6 dimensions de commandes

%initialisation des bruit de commande et de capteur
ekf.Q = diag([gyr_rms(1),gyr_rms(2),gyr_rms(3),acc_rms(1),acc_rms(2),acc_rms(3)]);
ekf.R = diag([f_sim*(0.1)^2, f_sim*(0.1)^2, f_sim*(0.1)^2]);

% Initialisation de l'état et de la covariance initiale
%X_est = [x y z qw qx qy qz vx vy vz abx aby abz ao11 ao12 ao13 ao21 ao22
%ao23 ao31 ao32 ao33 gbx gby gbz go11 go12 go13 go21 go22 go23 go31 go32 go33 g]
ekf.Xn = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0;0;0;0; acc_axis_alignement_inv(1,1); acc_axis_alignement_inv(1,2); acc_axis_alignement_inv(1,3); acc_axis_alignement_inv(2,1); acc_axis_alignement_inv(2,2); acc_axis_alignement_inv(2,3); acc_axis_alignement_inv(3,1); acc_axis_alignement_inv(3,2); acc_axis_alignement_inv(3,3);gyr_bias_correction(1); gyr_bias_correction(2); gyr_bias_correction(3); gyr_axis_alignement_inv(1,1); gyr_axis_alignement_inv(1,2); gyr_axis_alignement_inv(1,3); gyr_axis_alignement_inv(2,1); gyr_axis_alignement_inv(2,2); gyr_axis_alignement_inv(2,3); gyr_axis_alignement_inv(3,1); gyr_axis_alignement_inv(3,2); gyr_axis_alignement_inv(3,3);9.81]; % état initial;
ekf.Pn(1:3,1:3) = 0;
ekf.Pn(4:10,4:10) = 0;
ekf.Pn(14:22,14:22) = eye(9)*(acc_cross_tolerance)^2;
ekf.Pn(26:34,26:34) = eye(9)*(gyr_cross_tolerance)^2;
ekf.Pn(11,11) = acc_xy_bias_tolerance^2;
ekf.Pn(12,12) = acc_xy_bias_tolerance^2;
ekf.Pn(13,13) = acc_z_bias_tolerance^2;
ekf.Pn(23,23) = gyr_rms(1)/n_static_cal;
ekf.Pn(23,23) = gyr_rms(2)/n_static_cal;
ekf.Pn(23,23) = gyr_rms(3)/n_static_cal;

ekf.Pn(35,35) = 0.02^2;



X_est = zeros([length(t),35]);
P_est = zeros([length(t),35,35]);

for i = 1:length(t)
     % Prédiction de l'état
    current_t = t(i)
    ekf.Un = [gyr_xx(i); gyr_yy(i); gyr_zz(i); acc_xx(i); acc_yy(i); acc_zz(i)];
    ekf = ekf.predict();
    ekf.Zn = [gps_x(i);gps_y(i);gps_z(i)];% le système reste proche de l'origine
    ekf = ekf.update();
    X_est(i,:) = ekf.Xn;
    P_est(i,:,:) = ekf.Pn;
end

figure('Name', 'P');
image(abs(sqrt(ekf.Pn)),'CDataMapping','scaled');
colorbar;

acc_bias_correction = ekf.Xn(11:13);
acc_non_orthogonality_correction=reshape(ekf.Xn(14:22),[3 3]);
gyr_bias_correction =  ekf.Xn(23:25);
gyr_non_orthogonality_correction = reshape(ekf.Xn(26:34),[3 3]);


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
plot(t, X_est(:,8), 'r--', 'LineWidth', 1.5); % v_x estimé
plot(t, X_est(:,9), 'g--', 'LineWidth', 1.5); % v_y estimé
plot(t, X_est(:,10), 'b--', 'LineWidth', 1.5); % v_z estimé
plot(t, vx, 'r-', 'LineWidth', 1.5); % v_x réel en pointillé
plot(t, vy, 'g-', 'LineWidth', 1.5); % v_y réel en pointillé
plot(t, vz, 'b-', 'LineWidth', 1.5); % v_z réel en pointillé

% Incertitudes en pointillé
plot(t, X_est(:,8) + sqrt(squeeze(P_est(:,8,8))), 'r--', 'LineWidth', 0.5); % v_x + sigma
plot(t, X_est(:,8) - sqrt(squeeze(P_est(:,8,8))), 'r--', 'LineWidth', 0.5); % v_x - sigma
plot(t, X_est(:,9) + sqrt(squeeze(P_est(:,9,9))), 'g--', 'LineWidth', 0.5); % v_y + sigma
plot(t, X_est(:,9) - sqrt(squeeze(P_est(:,9,9))), 'g--', 'LineWidth', 0.5); % v_y - sigma
plot(t, X_est(:,10) + sqrt(squeeze(P_est(:,10,10))), 'b--', 'LineWidth', 0.5); % v_z + sigma
plot(t, X_est(:,10) - sqrt(squeeze(P_est(:,10,10))), 'b--', 'LineWidth', 0.5); % v_z - sigma


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

plot(t, X_est(:,4), 'r--', 'LineWidth', 1.5); % qw estimé
plot(t, X_est(:,5), 'g--', 'LineWidth', 1.5); % qx estimé
plot(t, X_est(:,6), 'b--', 'LineWidth', 1.5); % qy estimé
plot(t, X_est(:,7), 'k--', 'LineWidth', 1.5); % qz estimé

% Incertitudes en pointillé
plot(t, X_est(:,4) + sqrt(squeeze(P_est(:,4,4))), 'r--', 'LineWidth', 0.5); % qw + sigma
plot(t, X_est(:,4) - sqrt(squeeze(P_est(:,4,4))), 'r--', 'LineWidth', 0.5); % qw - sigma
plot(t, X_est(:,5) + sqrt(squeeze(P_est(:,5,5))), 'g--', 'LineWidth', 0.5); % qx + sigma
plot(t, X_est(:,5) - sqrt(squeeze(P_est(:,5,5))), 'g--', 'LineWidth', 0.5); % qx - sigma
plot(t, X_est(:,6) + sqrt(squeeze(P_est(:,6,6))), 'b--', 'LineWidth', 0.5); % qy + sigma
plot(t, X_est(:,6) - sqrt(squeeze(P_est(:,6,6))), 'b--', 'LineWidth', 0.5); % qy - sigma
plot(t, X_est(:,7) + sqrt(squeeze(P_est(:,7,7))), 'k--', 'LineWidth', 0.5); % qz + sigma
plot(t, X_est(:,7) - sqrt(squeeze(P_est(:,7,7))), 'k--', 'LineWidth', 0.5); % qz - sigma

title('Quaternion d''Orientation Estimé vs réel avec Incertitudes');
legend('qw', 'qx', 'qy', 'qz', 'qw estimé', 'qx estimé', 'qy estimé', 'qz estimé');
xlabel('Temps');
ylabel('Valeur');
grid on;
hold off;



% Supposons que 't' représente le vecteur des instants de temps
% et que 'X_est', 'P_est', 'acc_bias_correction', 'gyr_bias_correction',
% 'acc_non_orthogonality_correction', et 'gyr_cross_tolerance' sont définis

% Nombre total d'itérations
num_iterations = length(t);

% Créer une figure avec plusieurs sous-graphiques pour les biais
figure('Name',"Bias accéléromètres et gyroscopes");

% Biais de l'accéléromètre
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

% Biais du gyroscope
subplot(2, 1, 2);
hold on;
plot(t, gyr_bias(1)*ones([1 length(t)]), 'r-', 'LineWidth', 1); % Ligne pour la valeur vraie
plot(t, gyr_bias(2)*ones([1 length(t)]), 'g-', 'LineWidth', 1); % Ligne pour la valeur vraie
plot(t, gyr_bias(3)*ones([1 length(t)]), 'b-', 'LineWidth', 1); % Ligne pour la valeur vraie

plot(t, -X_est(:,23)+sqrt(squeeze(P_est(:,23,23))), 'r--', 'LineWidth', 0.5); % qw + sigma
plot(t, -X_est(:,23)-sqrt(squeeze(P_est(:,23,23))), 'r--', 'LineWidth', 0.5); % qw + sigma

plot(t, -X_est(:,24)+sqrt(squeeze(P_est(:,24,24))), 'g--', 'LineWidth', 0.5); % qw + sigma
plot(t, -X_est(:,24)-sqrt(squeeze(P_est(:,24,24))), 'g--', 'LineWidth', 0.5); % qw + sigma


plot(t, -X_est(:,25)+sqrt(squeeze(P_est(:,25,25))), 'b--', 'LineWidth', 0.5); % qw + sigma
plot(t, -X_est(:,25)-sqrt(squeeze(P_est(:,25,25))), 'b--', 'LineWidth', 0.5); % qw + sigma

title('Biais du Gyroscope');
legend('Biais vrai (X)', 'Biais vrai (Y)', 'Biais vrai (Z)', 'Estimation X', 'Estimation Y', 'Estimation Z');
grid on;

% Créer une nouvelle figure avec plusieurs sous-graphiques
figure("Name",'Évolution de la Matrice de Non-Orthogonalité des Accéléromètres');

% Matrice de non-orthogonalité de l'accéléromètre
acc_non_orthogonality_inv = inv(acc_non_orthogonality);
for i = 1:3
    for j=1:3
        subplot(3, 3, (i-1)*3+j);
        hold on;
        plot(t, acc_non_orthogonality_inv(i, j) * ones(size(t)), '--', 'LineWidth', 1); % Ligne pour la valeur vraie
        plot(t, X_est(:,13 + (i-1)*3+j), 'b--', 'LineWidth', 1); % qw + sigma
        plot(t, X_est(:,13 + (i-1)*3+j)+sqrt(squeeze(P_est(:,13 + (i-1)*3+j,13 + (i-1)*3+j))), 'b--', 'LineWidth', 0.5); % qw + sigma
        plot(t, X_est(:,13 + (i-1)*3+j)-sqrt(squeeze(P_est(:,13 + (i-1)*3+j,13 + (i-1)*3+j))), 'b--', 'LineWidth', 0.5); % qw + sigma
        title(['ao' num2str(i) num2str(j)]);
        grid on;
    end
end

% Créer une nouvelle figure avec plusieurs sous-graphiques
figure("Name",'Évolution de la Matrice de Non-Orthogonalité des gyroscopes');


% Matrice de non-orthogonalité du gyroscope
gyr_non_orthogonality_inv = inv(gyr_non_orthogonality);
for i = 1:3
    for j=1:3
        subplot(3, 3, (i-1)*3+j);
        hold on;
        plot(t, gyr_non_orthogonality_inv(i, j) * ones(size(t)), '--', 'LineWidth', 1); % Ligne pour la valeur vraie
        plot(t, X_est(:,25 + (i-1)*3+j), 'b--', 'LineWidth', 1); % qw + sigma
        plot(t, X_est(:,25 + (i-1)*3+j)+sqrt(squeeze(P_est(:,25 + (i-1)*3+j,25 + (i-1)*3+j))), 'b--', 'LineWidth', 0.5); % qw + sigma
        plot(t, X_est(:,25 + (i-1)*3+j)-sqrt(squeeze(P_est(:,25 + (i-1)*3+j,25 + (i-1)*3+j))), 'b--', 'LineWidth', 0.5); % qw + sigma
        title(['go' num2str(i) num2str(j)]);
        grid on;
    end
end



figure('Name',"intensité de pesenteur");
plot(t,abs(gravity)*ones([1 length(t)]), 'r-', 'LineWidth', 1.5);
hold on;
plot(t, X_est(:,35), 'r--', 'LineWidth', 1.5); % qw
plot(t, X_est(:,35) + sqrt(squeeze(P_est(:,35,35))), 'r--', 'LineWidth', 0.5); % qw + sigma
plot(t, X_est(:,35) - sqrt(squeeze(P_est(:,35,35))), 'r--', 'LineWidth', 0.5); % qw + sigma

title('Intensité de pesenteur Estimée vs réel avec Incertitudes');
legend('g','g estimé');
xlabel('Temps');
ylabel('Valeur');
grid on;
hold off;

%{
% Biais du gyroscope
subplot(2, 1, 2);
errorbar(t, X_est(:,23:25), sqrt(squeeze(P_est(:,23:25,23:25))), 'LineWidth', 2);
title('Biais du Gyroscope');
legend('X', 'Y', 'Z');
grid on;

% Créer une figure avec plusieurs sous-graphiques pour les matrices de non-orthogonalité
figure;

% Matrice de non-orthogonalité de l'accéléromètre
subplot(2, 1, 1);
errorbar(t, X_est(:,14:22), sqrt(squeeze(P_est(:,14:22,14:22))), 'LineWidth', 2);
title('Matrice de Non-Orthogonalité de l''Accéléromètre');
legend('ao11', 'ao12', 'ao13', 'ao21', 'ao22', 'ao23', 'ao31', 'ao32', 'ao33');
grid on;

% Matrice de non-orthogonalité du gyroscope
subplot(2, 1, 2);
errorbar(t, X_est(:,26:34), sqrt(squeeze(P_est(:,26:34,26:34))), 'LineWidth', 2);
title('Matrice de Non-Orthogonalité du Gyroscope');
legend('go11', 'go12', 'go13', 'go21', 'go22', 'go23', 'go31', 'go32', 'go33');
grid on;
%}

function next_X = transitionFunction(prev_X, U)
        %X_est = [x y z qw qx qy qz vx vy vz abx aby abz ao11 ao12 ao13 ao21 ao22
        %ao23 ao31 ao32 ao33 gbx gby gbz go11 go12 go13 go21 go22 go23 go31 go32 go33] -> length = 34
        %U = [gx gy gz ax ay az]
        %fonction de transition d'état (pour la prédiction à l'aide des
        %commandes ou capteurs proprioceptifs)
        
        dt = 0.01;
        [x,y,z,qw,qx,qy,qz,vx,vy,vz,abx,aby,abz,ao11,ao12,ao13,ao21,ao22,ao23,ao31,ao32,ao33,gbx,gby,gbz,go11,go12,go13,go21,go22,go23,go31,go32,go33,g] = deal(prev_X(1),prev_X(2),prev_X(3),prev_X(4),prev_X(5),prev_X(6),prev_X(7),prev_X(8),prev_X(9),prev_X(10),prev_X(11),prev_X(12),prev_X(13),prev_X(14),prev_X(15),prev_X(16),prev_X(17),prev_X(18),prev_X(19),prev_X(20),prev_X(21),prev_X(22),prev_X(23),prev_X(24),prev_X(25),prev_X(26),prev_X(27),prev_X(28),prev_X(29),prev_X(30),prev_X(31),prev_X(32),prev_X(33),prev_X(34),prev_X(35));
        [gx,gy,gz,ax,ay,az] = deal(U(1),U(2),U(3),U(4),U(5),U(6));
        
        x_ = x+vx*dt;
	    y_ = y+vy*dt;
	    z_ = z+vz*dt;					% integration des vitesses linéaires

        ac_xyz = [ao11 ao12 ao13;ao21 ao22 ao23;ao31 ao32 ao33]*([ax;ay;az]+[abx;aby;abz]);
        [ae_x,ae_y,ae_z] = rotate(ac_xyz(1),ac_xyz(2),ac_xyz(3),qw,qx,qy,qz);
        vx_ = vx+ae_x*dt;
	    vy_ = vy+ae_y*dt;
	    vz_ = vz+(ae_z+g)*dt;				% integration des accélération linéaires
        
    
    


        gc_xyz = [go11 go12 go13;go21 go22 go23;go31 go32 go33]*([gx;gy;gz]+[gbx;gby;gbz]);	
      
        [dqw,dqx,dqy,dqz] = quatMul(qw,qx,qy,qz,0,gc_xyz(1),gc_xyz(2),gc_xyz(3));
        q_ = [qw,qx,qy,qz] + 0.5*dt*[dqw,dqx,dqy,dqz];
        [qw_,qx_,qy_,qz_] = quatNormalize(q_(1),q_(2),q_(3),q_(4));

        next_X = [x_;y_;z_;qw_;qx_;qy_;qz_;vx_;vy_;vz_;abx;aby;abz;ao11;ao12;ao13;ao21;ao22;ao23;ao31;ao32;ao33;gbx;gby;gbz;go11;go12;go13;go21;go22;go23;go31;go32;go33;g];
end

function Z_predict = measurementFunction(X)
        %fonction pour prédire les donnés capteurs exteroceptifs (gps)
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

function [q2w,q2x,q2y,q2z] = quatConj(q1w,q1x,q1y,q1z)
    q2w=q1w;
    q2x=-q1x;
    q2y=-q1y;
    q2z=-q1z;
end

function [q2w,q2x,q2y,q2z] = quatNormalize(q1w,q1x,q1y,q1z)
    norm = sqrt(q1w^2+q1x^2+q1y^2+q1z^2);
    q2w = q1w/norm;
    q2x = q1x/norm;
    q2y = q1y/norm;
    q2z = q1z/norm;
end