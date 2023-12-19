function [q3w,q3x,q3y,q3z] = quatMul(q1w,q1x,q1y,q1z,q2w,q2x,q2y,q2z)
    q3w = q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z;
    q3x = q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y;
    q3y = q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x;
    q3z = q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w;
end