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
#define ACC_VAR 2
#define EXT_VAR 1
#define INIT_ATT_VAR 1e-6

void f(const Vector &x, const Vector &u, const Vector &c)
{
        data_type dt = c.data[0];
        data_type q1 = x.data[0], q2 = x.data[1], q3 = x.data[2], q4 = x.data[3];
        data_type gx = u.data[0], gy = u.data[1], gz = u.data[2];
        data_type dq1 = dt*(- q2*gx - q3*gy - q4*gz) / 2;
        data_type dq2 = dt*(q1*gx + q3*gz - q4*gy) / 2;
        data_type dq3 = dt*(q1*gy - q2*gz + q4*gx) / 2;
        data_type dq4 = dt*(q1*gz + q2*gy - q3*gx) / 2;
        Ekf::feedV.data[0] = q1 + dq1;
        Ekf::feedV.data[1] = q2 + dq2;
        Ekf::feedV.data[2] = q3 + dq3;
        Ekf::feedV.data[3] = q4 + dq4;
        data_type norm_inv = 1 / sqrt(Ekf::feedV.data[0] * Ekf::feedV.data[0] + Ekf::feedV.data[1] * Ekf::feedV.data[1] + Ekf::feedV.data[2] * Ekf::feedV.data[2] + Ekf::feedV.data[3] * Ekf::feedV.data[3]);
        Ekf::feedV.data[0] *= norm_inv;
        Ekf::feedV.data[1] *= norm_inv;
        Ekf::feedV.data[2] *= norm_inv;
        Ekf::feedV.data[3] *= norm_inv;

}

// jacobienne de f par rapport à x
void Fx(const Vector &x, const Vector &u, const Vector &c)
{
        data_type dt = c.data[0];
        Ekf::feedM.data[0] = 1;
        Ekf::feedM.data[1] = -dt * u.data[0] / 2;
        Ekf::feedM.data[2] = -dt * u.data[1] / 2;
        Ekf::feedM.data[3] = -dt * u.data[2] / 2;
        Ekf::feedM.data[4] = dt * u.data[0] / 2;
        Ekf::feedM.data[5] = 1;
        Ekf::feedM.data[6] = dt * u.data[2] / 2;
        Ekf::feedM.data[7] = -dt * u.data[1] / 2;
        Ekf::feedM.data[8] = dt * u.data[1] / 2;
        Ekf::feedM.data[9] = -dt * u.data[2] / 2;
        Ekf::feedM.data[10] = 1;
        Ekf::feedM.data[11] = dt * u.data[0] / 2;
        Ekf::feedM.data[12] = dt * u.data[2] / 2;
        Ekf::feedM.data[13] = dt * u.data[1] / 2;
        Ekf::feedM.data[14] = -dt * u.data[0] / 2;
        Ekf::feedM.data[15] = 1;
}

// jacobienne de f par rapport à u
void Fu(const Vector &x, const Vector &u, const Vector &c)
{
        data_type dt = c.data[0];
        Ekf::feedM.data[0] = -dt * x.data[1] / 2;
        Ekf::feedM.data[1] = -dt * x.data[2] / 2;
        Ekf::feedM.data[2] = -dt * x.data[3] / 2;
        Ekf::feedM.data[3] = dt * x.data[0] / 2;
        Ekf::feedM.data[4] = dt * x.data[3] / 2;
        Ekf::feedM.data[5] = -dt * x.data[0] / 2;
        Ekf::feedM.data[6] = dt * x.data[1] / 2;
        Ekf::feedM.data[7] = dt * x.data[2] / 2;
        Ekf::feedM.data[8] = -dt * x.data[0] / 2;
        Ekf::feedM.data[9] = dt * x.data[1] / 2;
        Ekf::feedM.data[10] = -dt * x.data[2] / 2;
        Ekf::feedM.data[11] = -dt * x.data[3] / 2;
}

// fonction de mesure
void h(const Vector &x, const Vector &c)
{
        // rotation du vecteur gravité par le conjugué de l'orientation
        data_type t2, t3, t5, t7, t8, t9;
        data_type q0 = x.data[0], q1 = -x.data[1], q2 = -x.data[2], q3 = -x.data[3];
        t2 = q0 * q1;
        t3 = q0 * q2;
        t5 = -q1 * q1;
        t7 = q1 * q3;
        t8 = -q2 * q2;
        t9 = q2 * q3;
        data_type g = -c.data[1];
        Ekf::feedV.data[0] = 2 * ((t3 + t7) * g);
        Ekf::feedV.data[1] = 2 * ((t9 - t2) * g);
        Ekf::feedV.data[2] = 2 * ((t5 + t8) * g) + g;
        // Ekf::feedV.data[2] = 2 * ((t5 - t8) * g) + g;
}

// // jacobienne de h par rapport à x
// void H(const Vector &x, const Vector &c)
// {
//         data_type q0 = x.data[0], q1 = -x.data[1], q2 = -x.data[2], q3 = -x.data[3];
//         data_type g2 = -2*c.data[1];
//         Ekf::feedM.data[0] = g2 * q2;
//         Ekf::feedM.data[1] = 0;
// }

void Dasta::configureStateEstimate()
{
        estimator.ekf.alloc(X_DIM, U_DIM, 1);
        estimator.ekf.setTransitionFunction(f);
        // estimator.ekf.setJacobianFunction_Fx(Fx);
        // estimator.ekf.setJacobianFunction_Fu(Fu);
        estimator.ekf.setMeasurementFunction(h, Z_DIM);

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
