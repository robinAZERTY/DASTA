#include "Quaternion.hpp"

Quaternion::Quaternion() : Vector()
{
    this->size = 4;
}

Quaternion::Quaternion(data_type w, data_type x, data_type y, data_type z) : Vector(4)
{
    (*this)(0) = w;
    (*this)(1) = x;
    (*this)(2) = y;
    (*this)(3) = z;
}

Quaternion::Quaternion(data_type *data) : Vector()
{
    this->size = 4;
    this->data = data;
}

void Quaternion::conjugate()
{
    data[1] = -data[1];
    data[2] = -data[2];
    data[3] = -data[3];
}

data_type norm_square(Vector &v)
{
    data_type norm = 0;
    for (int i = 0; i < v.size; i++)
        norm += v.data[i] * v.data[i];
    return norm;
}

void Quaternion::normalize()
{
    data_type norm_inv = 1 / sqrt(norm_square(*this));
    data[0] *= norm_inv;
    data[1] *= norm_inv;
    data[2] *= norm_inv;
    data[3] *= norm_inv;
}

// void Quaternion::to_angle_axis(Vector &res)
// {
//     res.data[0] = acos(data[0]) * 2;
//     if (abs(res.data[0]) < 1e-6)
//     {
//         res.data[1] = 0;
//         res.data[2] = 0;
//         res.data[3] = 0;
//         return;
//     }
//     data_type sin_angle = sin(res.data[0] / 2);
//     res.data[1] = data[1] / sin_angle;
//     res.data[2] = data[2] / sin_angle;
//     res.data[3] = data[3] / sin_angle;
// }

void mul(Quaternion &res, const Quaternion &q1, const Quaternion &q2)
{
    res.data[0] = q1.data[0] * q2.data[0] - q1.data[1] * q2.data[1] - q1.data[2] * q2.data[2] - q1.data[3] * q2.data[3];
    res.data[1] = q1.data[0] * q2.data[1] + q1.data[1] * q2.data[0] + q1.data[2] * q2.data[3] - q1.data[3] * q2.data[2];
    res.data[2] = q1.data[0] * q2.data[2] - q1.data[1] * q2.data[3] + q1.data[2] * q2.data[0] + q1.data[3] * q2.data[1];
    res.data[3] = q1.data[0] * q2.data[3] + q1.data[1] * q2.data[2] - q1.data[2] * q2.data[1] + q1.data[3] * q2.data[0];
}

void rotate(Vector &res, const Quaternion &q, const Vector &v)
{
    data_type t2, t3, t4, t5, t6, t7, t8, t9, t10;
    t2 = q.data[0] * q.data[1];
    t3 = q.data[0] * q.data[2];
    t4 = q.data[0] * q.data[3];
    t5 = -q.data[1] * q.data[1];
    t6 = q.data[1] * q.data[2];
    t7 = q.data[1] * q.data[3];
    t8 = -q.data[2] * q.data[2];
    t9 = q.data[2] * q.data[3];
    t10 = -q.data[3] * q.data[3];

    res.data[0] = 2 * ((t8 + t10) * v.data[0] + (t6 - t4) * v.data[1] + (t3 + t7) * v.data[2]) + v.data[0];
    res.data[1] = 2 * ((t4 + t6) * v.data[0] + (t5 + t10) * v.data[1] + (t9 - t2) * v.data[2]) + v.data[1];
    res.data[2] = 2 * ((t7 - t3) * v.data[0] + (t2 + t9) * v.data[1] + (t5 + t8) * v.data[2]) + v.data[2];
}

void q2rpy(Vector &res, const Quaternion &q)
{
    res.data[2] = atan2(2 * (q.data[0] * q.data[3] + q.data[1] * q.data[2]), 1 - 2 * (q.data[2] * q.data[2] + q.data[3] * q.data[3])); // yaw
    const data_type sinp = 2 * (q.data[0] * q.data[2] - q.data[3] * q.data[1]);
    if (sinp > 1 - 1e-6)
        res.data[1] = M_PI_2; // use 90 degrees if out of range
    else if (sinp < -1 + 1e-6)
        res.data[1] = -M_PI_2; // use -90 degrees if out of range
    else
        res.data[1] = asin(sinp); // pitch

    res.data[0] = atan2(2 * (q.data[0] * q.data[1] + q.data[2] * q.data[3]), 1 - 2 * (q.data[1] * q.data[1] + q.data[2] * q.data[2])); // roll
}

void rpy2q(Quaternion &res, const Vector &rpy)
{
    data_type cy = cos(rpy.data[2] * 0.5);
    data_type sy = sin(rpy.data[2] * 0.5);
    data_type cp = cos(rpy.data[1] * 0.5);
    data_type sp = sin(rpy.data[1] * 0.5);
    data_type cr = cos(rpy.data[0] * 0.5);
    data_type sr = sin(rpy.data[0] * 0.5);

    res.data[0] = cr * cp * cy + sr * sp * sy;
    res.data[1] = sr * cp * cy - cr * sp * sy;
    res.data[2] = cr * sp * cy + sr * cp * sy;
    res.data[3] = cr * cp * sy - sr * sp * cy;
}

void pow(Quaternion &res, const Quaternion &q, data_type n)
{
    data_type angle = acos(q.data[0]);
    data_type sin_angle = sin(angle);
    data_type angle_n = angle * n;
    res.data[0] = cos(angle_n);
    res.data[1] = q.data[1] * sin(angle_n) / sin_angle;
    res.data[2] = q.data[2] * sin(angle_n) / sin_angle;
    res.data[3] = q.data[3] * sin(angle_n) / sin_angle;
}