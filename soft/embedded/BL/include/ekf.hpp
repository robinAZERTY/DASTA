#ifndef EKF_HPP
#define EKF_HPP

#include "matrix.hpp"

typedef void (*Matrix_f2)(Vector&, Vector&);
typedef void (*Matrix_f1)(Vector&);

class Ekf
{
     protected:
        Matrix_f2 f;//transition function
        Matrix_f1 h;//observation function
        Matrix_f2 Fx;//jacobian of transition function
        Matrix_f2 Fu;//jacobian of transition function
        Matrix_f1 H;//jacobian of observation function
        Matrix *Fx_val;//value of jacobian of transition function
        Matrix *Fu_val;//value of jacobian of transition function
        Matrix *H_val;//value of jacobian of observation function
        Matrix *K;//kalman gain
        Matrix *S;//innovation covariance
        Matrix *I;//identity matrix
        Vector *y;//innovation
        Vector *xtmp;//temporary vector
        int x_dim;//dimension of state vector
        int z_dim;//dimension of observation vector
        int u_dim;//dimension of control vector

    public:
        static Matrix *tmp1,*tmp2, *Min; //temporary matrices
        static Matrix refP; //reference matrix for symmetrisation of P
        static Vector *Vin; //temporary vector for function return
        Ekf(Matrix_f2 f, Matrix_f1 h, int x_dim, int z_dim, int u_dim, Matrix_f2 Fx = nullptr, Matrix_f2 Fu = nullptr, Matrix_f1 H = nullptr);
        ~Ekf();

        Vector *u;//command
        Vector*x;//state
        void predict();

        Vector *z;//observation
        Matrix *R;//observation covariance
        Matrix *P;//state covariance
        Matrix *Q;//process covariance
        void update();

};
#ifndef ARDUINO
#include "ekf.cpp"
#endif
#endif