#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include "matrix.hpp"

// Quaternion class to represent 3D rotations

class Quaternion : public Vector{
    public:
        Quaternion();
        Quaternion(data_type w, data_type x, data_type y, data_type z);
        Quaternion(data_type *data);
        ~Quaternion() = default;
        void conjugate();
        void normalize();
        // void to_angle_axis(Vector &res);
};

void mul(Quaternion &res, const Quaternion &q1, const Quaternion &q2);
void rotate(Vector &res, const Quaternion &q, const Vector &v);
void q2rpy(Vector &res, const Quaternion &q);
void pow(Quaternion &res, const Quaternion &q, data_type n);
#ifndef ARDUINO
#include "quaternion.cpp"
#endif

#endif