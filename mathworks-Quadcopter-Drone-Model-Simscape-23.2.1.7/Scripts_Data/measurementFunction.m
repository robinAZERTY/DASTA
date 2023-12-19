function Z_predict = measurementFunction(X)
        %function to predict exteroceptive sensor data
        %Z_predict = [ir1x;ir1y;ir2x;ir2y];
        [x,y,z,qw,qx,qy,qz] = deal(X(1),X(2),X(3),X(4),X(5),X(6),X(7));
        %sensors settings
        cam_q= [0, 0, -1,0]; %camera orientation
        cam_p = [0, 0, 2.8]; %camera position
        cam_k=138.565; % constant gain for xy to pixel conversion
        ir1_p = [0.1 0 0]; % position of led1
        ir2_p = [-0.1 0 0]; % position of led2
        
        % perspective projection for led1
        [l1x,l1y,l1z] = rotateQ(ir1_p(1),ir1_p(2),ir1_p(3),qw,qx,qy,qz);
        [l1x,l1y,l1z] = rotateQ(l1x+x,l1y+y,l1z+z,cam_q(1),cam_q(2),cam_q(3),cam_q(4));
        ir1 = [l1x+cam_p(1),l1y+cam_p(2)]*cam_k/(l1z+cam_p(3));

        % perspective projection for led2
        [l2x,l2y,l2z] = rotateQ(ir2_p(1),ir2_p(2),ir2_p(3),qw,qx,qy,qz);
        [l2x,l2y,l2z] = rotateQ(l2x+x,l2y+y,l2z+z,cam_q(1),cam_q(2),cam_q(3),cam_q(4));
        ir2 = [l2x+cam_p(1),l2y+cam_p(2)]*cam_k/(l2z+cam_p(3));

        Z_predict = [ir1(1);ir1(2);ir2(1);ir2(2)]; 
end

function [x2,y2,z2] = rotateQ(x1,y1,z1,qw,qx,qy,qz)
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