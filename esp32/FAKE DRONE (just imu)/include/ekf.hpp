#ifndef EKF_HPP
#define EKF_HPP

#include "matrix.hpp"

typedef void (*Matrix_f2)(const Vector &, const Vector &);
typedef void (*Matrix_f1)(const Vector &);

class Ekf
{
protected:
    Matrix_f2 f;
    Matrix_f1 *h;
    Matrix_f2 Fx;
    Matrix_f2 Fu;
    Matrix_f1 *H;
    Matrix *Fx_val, *Fu_val, *H_val, *K, *S, *I;
    Vector *y, *utmp;
    uint_fast8_t x_dim;
    uint_fast8_t *z_dim;
    uint_fast8_t z_number;
    uint_fast8_t u_dim;

    void finite_diff_Fx(const uint8_t i, const data_type eps = 1e-4);
    void finite_diff_Fu(const uint8_t i, const data_type eps = 1e-4);
    void finite_diff_H(const uint8_t iz, const uint8_t i, const data_type eps = 1e-4);

    void finite_diff_Fx();
    void finite_diff_Fu();
    void finite_diff_H(const uint8_t iz = 0);

public:
    static Matrix *tmp1, *tmp2, *Min;
    // static Matrix refP;
    static Vector *Vin;
    Ekf(Matrix_f2 f, Matrix_f1 h[], uint_fast8_t x_dim, uint_fast8_t z_dim[], uint_fast8_t u_dim, Matrix_f2 Fx = nullptr, Matrix_f2 Fu = nullptr, Matrix_f1 H[] = nullptr, uint_fast8_t z_num = 1);
    ~Ekf();

    Vector *u, *x; // vectors
    void predict();

    Vector **z, **h_val; // vector

    Matrix **R, *P, *Q;
    void update(const uint8_t iz = 0);
};
#endif