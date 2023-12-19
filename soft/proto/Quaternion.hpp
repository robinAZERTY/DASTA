#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include "Matrix.hpp"

// Quaternion class to represent 3D rotations

class Quaternion : public Vector{
    public:
        Quaternion();
        Quaternion(data_type w, data_type x, data_type y, data_type z);
        ~Quaternion() = default;
        void conjugate();
        void normalize();
};

void mul(Quaternion &res, const Quaternion &q, const data_type s);
void mul(Quaternion &res, const Quaternion &q1, const Quaternion &q2);
void rotate(Vector &res, const Quaternion &q, const Vector &v);
void integrate(Quaternion &res, const Quaternion &q, const Quaternion &s, const data_type &dt);
void to_ypr(Vector &res, const Quaternion &q);

#ifndef ARDUINO
#include "Quaternion.cpp"
#endif

#endif