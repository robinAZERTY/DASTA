#include "Dasta.hpp"

/*
____________STATE ESTIMATE VECTOR______________

        State                   unit        index
orientation(quaternion)                     1:4



___________COMMANDS VECTOR_____________________

        State                   unit        index
gyroscopes(xyz)                 rad/s       1:3

___________SENSORS VECTOR_____________________

        State                   unit       index
accelerometers(xyz)             m/s²        1:3


___________PARAMS VECTOR_____________________

        State                   unit       index
dt                              s            1
g                               m/s²         2

*/

#define X_DIM 4
#define Z_DIM 3
#define U_DIM 3
#define C_DIM 2

#define GYRO_VAR 0.001
#define ACC_VAR 0.01
#define EXT_VAR 1
#define INIT_ATT_VAR 1e-6

void f(const Vector &x, const Vector &u, const Vector &c)
{
        data_type dt = c.data[0];
        // integrate the quaternion
        Ekf::feedV.data[0] = x.data[0] - (dt *(x.data[1] * u.data[0] + x.data[2] * u.data[1] + x.data[3] * u.data[2]) / 2);
        Ekf::feedV.data[1] = x.data[1] + (dt * (x.data[0] * u.data[0] - x.data[3] * u.data[1] + x.data[2] * u.data[2]) / 2);
        Ekf::feedV.data[2] = x.data[2] + (dt * (x.data[3] * u.data[0] + x.data[0] * u.data[1] - x.data[1] * u.data[2]) / 2);
        Ekf::feedV.data[3] = x.data[3] + (dt * (x.data[2] * u.data[0] - x.data[1] * u.data[1] + x.data[0] * u.data[2]) / 2);

        // normalize the quaternion
        data_type norm_inv = 1 / sqrt(Ekf::feedV.data[0] * Ekf::feedV.data[0] + Ekf::feedV.data[1] * Ekf::feedV.data[1] + Ekf::feedV.data[2] * Ekf::feedV.data[2] + Ekf::feedV.data[3] * Ekf::feedV.data[3]);
        Ekf::feedV.data[0] *= norm_inv;
        Ekf::feedV.data[1] *= norm_inv;
        Ekf::feedV.data[2] *= norm_inv;
        Ekf::feedV.data[3] *= norm_inv;
}

// fonction de mesure
void h(const Vector &x, const Vector &c)
{
        // rotation du vecteur gravité par le conjugué de l'orientation
        data_type t2, t3, t4, t5, t6, t7, t8, t9, t10;
        data_type q0 = x.data[0], q1 = -x.data[1], q2 = -x.data[2], q3 = -x.data[3];
        t2 = q0 * q1;
        t3 = q0 * q2;
        t4 = q0 * q3;
        t5 = -q1 * q1;
        t6 = q1 * q2;
        t7 = q1 * q3;
        t8 = -q2 * q2;
        t9 = q2 * q3;
        t10 = -q3 * q3;
        data_type g = -c.data[1];
        Ekf::feedV.data[0] = 2 * ((t3 + t7) * g);
        Ekf::feedV.data[1] = 2 * ((t9 - t2) * g);
        Ekf::feedV.data[2] = 2 * ((t5 - t8) * g) + g;
}

// // // jacobienne de f par rapport à x
// void Fx(const Vector &x, const Vector &u, const Vector &c)
// {
// }

// // jacobienne de f par rapport à u
// void Fu(const Vector &x, const Vector &u, const Vector &c)
// {
// }

void Dasta::configureStateEstimate()
{
        estimator.ekf.alloc(X_DIM, U_DIM, 1);
        estimator.ekf.setTransitionFunction(f);
        estimator.ekf.setMeasurementFunction(h, Z_DIM,0);
        estimator.ekf.c.alloc(C_DIM);
        estimator.ekf.c.data[0] = 0.01;
        estimator.ekf.c.data[1] = 9.81;
        estimator.ekf.x.fill(0);
        estimator.ekf.x.data[0] = 1;
        estimator.ekf.P.fill(0);
        estimator.ekf.P(0, 0) = INIT_ATT_VAR;
        estimator.ekf.P(1, 1) = INIT_ATT_VAR;
        estimator.ekf.P(2, 2) = INIT_ATT_VAR;
        estimator.ekf.P(3, 3) = INIT_ATT_VAR;
        estimator.ekf.Q(0) = GYRO_VAR;
        estimator.ekf.Q(1) = GYRO_VAR;
        estimator.ekf.Q(2) = GYRO_VAR;
        estimator.ekf.R[0](0, 0) = ACC_VAR;
        estimator.ekf.R[0](1, 1) = ACC_VAR;
        estimator.ekf.R[0](2, 2) = ACC_VAR;
        estimator.orientation.data = estimator.ekf.x.data;
        estimator.dt_proprio = &estimator.ekf.c.data[0];
        estimator.gravity = &estimator.ekf.c.data[1];
}
