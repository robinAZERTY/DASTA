function next_X = transitionFunction(prev_X, U)
        %X_est = [x y z qw qx qy qz vx vy vz] -> position, orientation and linear velocity
        %U = [gx gy gz ax ay az] -> 
        %function for state transition (pour la prediction using commands
        %or proprioceptive sensors)
        gravityy = 9.80665;
        dtt = 0.01;
        [x,y,z,qw,qx,qy,qz,vx,vy,vz] = deal(prev_X(1),prev_X(2),prev_X(3),prev_X(4), ...
                                prev_X(5),prev_X(6),prev_X(7),prev_X(8),prev_X(9),prev_X(10));
        [gx,gy,gz,ax,ay,az] = deal(U(1),U(2),U(3),U(4),U(5),U(6));
        
        ac_xyz = [ax;ay;az];
        [ae_x,ae_y,ae_z] = rotate(ac_xyz(1),ac_xyz(2),ac_xyz(3),qw,-qx,-qy,-qz);
        vx_ = vx+ae_x*dtt;
	    vy_ = vy+ae_y*dtt;
	    vz_ = vz+(ae_z-gravityy)*dtt;				% integration of linear acceleration
        
        x_ = x+vx_*dtt;
	    y_ = y+vy_*dtt;
	    z_ = z+vz_*dtt;					% integration of linear velocity
    
        gc_xyz = [gx;gy;gz];	
        
        [dqw,dqx,dqy,dqz] = quatMul(qw,qx,qy,qz,0,gc_xyz(1),gc_xyz(2),gc_xyz(3));
        q_ = [qw,qx,qy,qz] + 0.5*dtt*[dqw,dqx,dqy,dqz];
        [qw_,qx_,qy_,qz_] = quatNormalize(q_(1),q_(2),q_(3),q_(4));

        next_X = [x_;y_;z_;qw_;qx_;qy_;qz_;vx_;vy_;vz_];
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