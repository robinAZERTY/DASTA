%{
____________STATE ESTIMATE VECTOR______________

        State                   unit        index
postion(xyz)                    m           1:3
velocity(xyz)                   m/s         4:6
orientation(quaternion)                     7:10



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

___________PARAMS VECTOR_____________________

        State                   unit       index
cam1_pos(xyz)                   m           1:3
cam1_ori(quaternion)                        4:7
cam1_k                          pixel        8
cam2_pos(xyz)                   m           9:11
cam2_ori(quaternion)                        12:15
cam2_k                          pixel        16
dt                              s            17
g                               m/s²         18
led1_pos(xyz)                   m           19:21
led2_pos(xyz)                   m           22:24
%}

X = sym ('X',[10,1]);
U = sym ('U',[6,1]);
C = sym ('C',[24,1]);

f = simplify(transitionFunction(X,U,C))

h = simplify(measurementFunctionPrime(X,C))

Jfx = simplify(jacobian(f,X))

Jfu = simplify(jacobian(f,U))

%Jhx = simplify(jacobian(h,X))
syms C4C5 C4C6 C4C7
syms C5C6 C5C7
syms C6C5 C6C7
subs()
%jacobian(h,X)

function next_X = transitionFunction(prev_X, U, C)

        %fonction de transition d'état (pour la prédiction à l'aide des
        %commandes ou capteurs proprioceptifs)
        
        dt = C(17);
        g=C(18);
        [x,y,z,vx,vy,vz,qw,qx,qy,qz] = deal(prev_X(1),prev_X(2),prev_X(3),prev_X(4),prev_X(5),prev_X(6),prev_X(7),prev_X(8),prev_X(9),prev_X(10));
        [gx,gy,gz,ax,ay,az] = deal(U(1),U(2),U(3),U(4),U(5),U(6));
       

        x_ = x+vx*dt;
	    y_ = y+vy*dt;
	    z_ = z+vz*dt;					% integration des vitesses linéaires

        [ae_x,ae_y,ae_z] = rotate(ax,ay,az,qw,qx,qy,qz);
        vx_ = vx+ae_x*dt;
	    vy_ = vy+ae_y*dt;
	    vz_ = vz+(ae_z-g)*dt;				% integration des accélération linéaires


        [dqw,dqx,dqy,dqz] = quatMul(qw,qx,qy,qz,0,gx,gy,gz);
        q_ = [qw,qx,qy,qz] + 0.5*dt*[dqw,dqx,dqy,dqz];
        [qw_,qx_,qy_,qz_] = quatNormalize(q_(1),q_(2),q_(3),q_(4));

        next_X = transpose([x_,y_,z_,vx_,vy_,vz_,qw_,qx_,qy_,qz_]);
end

function Z_predict = measurementFunction(X,C)
        %fonction pour prédire les donnés capteurs exteroceptifs
        %Z_predict = [X(1); X(2); X(3)];
        % perspective projection for led1
        %function to predict exteroceptive sensor data
        %Z_predict  : l1c1x, l1c1y, l2c1x, l2c1y, l1c2x, l1c2y, l2c2x, l2c2y
        [x,y,z,qw,qx,qy,qz] = deal(X(1),X(2),X(3),X(7),X(8),X(9),X(10));


        %sensors settings
        cam1_p = C(1:3);
        cam1_q = C(4:7);
        cam1_k = C(8);
        cam2_p = C(9:11);
        cam2_q = C(12:15);
        cam2_k = C(16);


        ir1_p = C(19:21); % position of led1
        ir2_p = C(22:24); % position of led2

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

function Z_predict = measurementFunctionPrime(X,C)
        %fonction pour prédire les donnés capteurs exteroceptifs
        %Z_predict = [X(1); X(2); X(3)];
        % perspective projection for led1
        %function to predict exteroceptive sensor data
        %Z_predict  : l1c1x, l1c1y, l2c1x, l2c1y, l1c2x, l1c2y, l2c2x, l2c2y
        [x,y,z,qw,qx,qy,qz] = deal(X(1),X(2),X(3),X(7),X(8),X(9),X(10));


        %sensors settings
        cam1_p = C(1:3);
        cam1_q = C(4:7);
        cam1_k = C(8);


        ir1_p = C(19:21); % position of led

        [l1x,l1y,l1z] = rotate(ir1_p(1),ir1_p(2),ir1_p(3),qw,-qx,-qy,-qz);
        l1x= l1x+x;
        l1y= l1y+y;
        l1z= l1z+z;
        [l1x,l1y,l1z] = rotate(l1x-cam1_p(1),l1y-cam1_p(2),l1z-cam1_p(3),cam1_q(1),-cam1_q(2),-cam1_q(3),-cam1_q(4));
        l1c1 = [l1x,l1y]*cam1_k/l1z;


        Z_predict = [l1x; l1y; l1z];
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
