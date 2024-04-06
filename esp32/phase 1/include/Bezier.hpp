#ifndef BEZIER_HPP
#define BEZIER_HPP

#include "quaternion.hpp" 

// slerp: spherical linear interpolation
void slerp(Quaternion &res, const Quaternion &q1, const Quaternion &q2, data_type t)
{
    data_type dot = q1(0) * q2(0) + q1(1) * q2(1) + q1(2) * q2(2) + q1(3) * q2(3);
    if (dot < 0)
    {
        dot = -dot;
        res(0) = -q2(0);
        res(1) = -q2(1);
        res(2) = -q2(2);
        res(3) = -q2(3);
    }
    else
    {
        res(0) = q2(0);
        res(1) = q2(1);
        res(2) = q2(2);
        res(3) = q2(3);
    }

    if (dot > 0.95)
    {
        res(0) = q1(0) + t * (res(0) - q1(0));
        res(1) = q1(1) + t * (res(1) - q1(1));
        res(2) = q1(2) + t * (res(2) - q1(2));
        res(3) = q1(3) + t * (res(3) - q1(3));
        res.normalize();
        return;
    }

    pow(res, q1, 1 - t);
}


class Squad // Spherical Quadrangle interpolation
{
    public:
        Quaternion q0, q1, a, b;
        Squad(const Quaternion &q0, const Quaternion &q1, const Quaternion &a, const Quaternion &b) : q0(q0), q1(q1), a(a), b(b) {}
        void interpolate(Quaternion &res, data_type t)
        {
            Quaternion slerp1, slerp2;
            slerp(slerp1, q0, q1, t);
            slerp(slerp2, a, b, t);
            slerp(res, slerp1, slerp2, 2 * t * (1 - t));
        }
};

#endif // BEZIER_HPP