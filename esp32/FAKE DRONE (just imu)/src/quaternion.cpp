#include "quaternion.hpp"

Quaternion::Quaternion():Vector(){
    this->size = 4;
    // (*this)(0) = 1;
    // (*this)(1) = 0;
    // (*this)(2) = 0;
    // (*this)(3) = 0;
}

Quaternion::Quaternion(data_type w, data_type x, data_type y, data_type z) : Vector(4){
    (*this)(0) = w;
    (*this)(1) = x;
    (*this)(2) = y;
    (*this)(3) = z;
}

void Quaternion::conjugate(){
    // (*this)(1) = -(*this)(1);
    // (*this)(2) = -(*this)(2);
    // (*this)(3) = -(*this)(3);
    //using data instead of method call to avoid overhead
    data[1] = -data[1];
    data[2] = -data[2];
    data[3] = -data[3];
}

data_type norm_square(Vector &v){
    data_type norm = 0;
    for(int i = 0; i < v.size; i++){
        // norm += v(i)*v(i);
        //using data instead of method call to avoid overhead
        norm += v.data[i]*v.data[i];
    }
    return norm;
}

void Quaternion::normalize(){
    data_type norm_inv = 1/sqrt(norm_square(*this));
    // (*this)(0) *= norm_inv;
    // (*this)(1) *= norm_inv;
    // (*this)(2) *= norm_inv;
    // (*this)(3) *= norm_inv;

    //using data instead of method call to avoid overhead
    data[0] *= norm_inv;
    data[1] *= norm_inv;
    data[2] *= norm_inv;
    data[3] *= norm_inv;
}

void mul(Quaternion &res, const Quaternion &q1, const Quaternion &q2){
    res(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
    res(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
    res(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
    res(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);
}


void rotate(Vector &res, const Quaternion &q, const Vector &v){
    /*
    [vx2,vy2,vz2] = rotateQ(vx, vy, vz,q1a, q1b, q1c, q1d)
    vx2 = q1a*(q1a*vx - q1d*vy + q1c*vz) + q1c*(q1b*vy - q1c*vx + q1a*vz) + q1b*(q1b*vx + q1c*vy + q1d*vz) - q1d*(q1d*vx + q1a*vy - q1b*vz)
    vy2 = q1a*(q1d*vx + q1a*vy - q1b*vz) - q1b*(q1b*vy - q1c*vx + q1a*vz) + q1c*(q1b*vx + q1c*vy + q1d*vz) + q1d*(q1a*vx - q1d*vy + q1c*vz) 
    vz2 = q1a*(q1b*vy - q1c*vx + q1a*vz) + q1b*(q1d*vx + q1a*vy - q1b*vz) - q1c*(q1a*vx - q1d*vy + q1c*vz) + q1d*(q1b*vx + q1c*vy + q1d*vz)
    */
    // res(0) = q(0)*(q(0)*v(0) - q(3)*v(1) + q(2)*v(2)) + q(2)*(q(1)*v(1) - q(2)*v(0) + q(0)*v(2)) + q(1)*(q(1)*v(0) + q(2)*v(1) + q(3)*v(2)) - q(3)*(q(3)*v(0) + q(0)*v(1) - q(1)*v(2));
    // res(1) = q(0)*(q(3)*v(0) + q(0)*v(1) - q(1)*v(2)) - q(1)*(q(1)*v(1) - q(2)*v(0) + q(0)*v(2)) + q(2)*(q(1)*v(0) + q(2)*v(1) + q(3)*v(2)) + q(3)*(q(0)*v(0) - q(3)*v(1) + q(2)*v(2));
    // res(2) = q(0)*(q(1)*v(1) - q(2)*v(0) + q(0)*v(2)) + q(1)*(q(3)*v(0) + q(0)*v(1) - q(1)*v(2)) - q(2)*(q(0)*v(0) - q(3)*v(1) + q(2)*v(2)) + q(3)*(q(1)*v(0) + q(2)*v(1) + q(3)*v(2));

    //using v.data[0] instead of v(0) to avoid method call overhead
    res.data[0] = q.data[0]*(q.data[0]*v.data[0] - q.data[3]*v.data[1] + q.data[2]*v.data[2]) + q.data[2]*(q.data[1]*v.data[1] - q.data[2]*v.data[0] + q.data[0]*v.data[2]) + q.data[1]*(q.data[1]*v.data[0] + q.data[2]*v.data[1] + q.data[3]*v.data[2]) - q.data[3]*(q.data[3]*v.data[0] + q.data[0]*v.data[1] - q.data[1]*v.data[2]);
    res.data[1] = q.data[0]*(q.data[3]*v.data[0] + q.data[0]*v.data[1] - q.data[1]*v.data[2]) - q.data[1]*(q.data[1]*v.data[1] - q.data[2]*v.data[0] + q.data[0]*v.data[2]) + q.data[2]*(q.data[1]*v.data[0] + q.data[2]*v.data[1] + q.data[3]*v.data[2]) + q.data[3]*(q.data[0]*v.data[0] - q.data[3]*v.data[1] + q.data[2]*v.data[2]);
    res.data[2] = q.data[0]*(q.data[1]*v.data[1] - q.data[2]*v.data[0] + q.data[0]*v.data[2]) + q.data[1]*(q.data[3]*v.data[0] + q.data[0]*v.data[1] - q.data[1]*v.data[2]) - q.data[2]*(q.data[0]*v.data[0] - q.data[3]*v.data[1] + q.data[2]*v.data[2]) + q.data[3]*(q.data[1]*v.data[0] + q.data[2]*v.data[1] + q.data[3]*v.data[2]);
}

void q2rpy(Vector &res, const Quaternion &q){
    // res(0) = atan2(2*(q(0)*q(1) + q(2)*q(3)), 1 - 2*(q(1)*q(1) + q(2)*q(2))); // roll
    // res(1) = asin(2*(q(0)*q(2) - q(3)*q(1))); // pitch
    // res(2) = atan2(2*(q(0)*q(3) + q(1)*q(2)), 1 - 2*(q(2)*q(2) + q(3)*q(3))); // yaw
    //using data instead of method call to avoid overhead
    res.data[0] = atan2(2*(q.data[0]*q.data[1] + q.data[2]*q.data[3]), 1 - 2*(q.data[1]*q.data[1] + q.data[2]*q.data[2])); // roll
    res.data[1] = asin(2*(q.data[0]*q.data[2] - q.data[3]*q.data[1])); // pitch
    res.data[2] = atan2(2*(q.data[0]*q.data[3] + q.data[1]*q.data[2]), 1 - 2*(q.data[2]*q.data[2] + q.data[3]*q.data[3])); // yaw
}