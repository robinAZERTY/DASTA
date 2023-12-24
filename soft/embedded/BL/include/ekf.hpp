#ifndef EKF_HPP
#define EKF_HPP

#include "matrix.hpp"

typedef void (*Matrix_f2)(Vector&, Vector&);
typedef void (*Matrix_f1)(Vector&);

class Ekf
{
     protected:
        Matrix_f2 f;
        Matrix_f1 h;
        Matrix_f2 Fx;
        Matrix_f2 Fu;
        Matrix_f1 H;
        Matrix *Fx_val, *Fu_val, *H_val, *K, *S, *I;
        Vector *y, *xtmp;
        int x_dim;
        int z_dim;
        int u_dim;

    public:
        static Matrix *tmp1,*tmp2, *Min;
        static Matrix refP;
        static Vector *Vin;
        Ekf(Matrix_f2 f, Matrix_f1 h, int x_dim, int z_dim, int u_dim, Matrix_f2 Fx = nullptr, Matrix_f2 Fu = nullptr, Matrix_f1 H = nullptr);
        ~Ekf();

        Vector *u,*x;//vectors
        void predict();

        Vector *z;//vector
        Matrix *R, *P, *Q;
        void update();

};
#ifndef ARDUINO
#include "ekf.cpp"
#endif
#endif