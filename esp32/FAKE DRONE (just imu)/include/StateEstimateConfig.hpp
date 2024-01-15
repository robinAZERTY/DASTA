#ifndef STATEESTIMATECONFIG_HPP
#define STATEESTIMATECONFIG_HPP
#include "Dasta.hpp"

/*
____________STATE ESTIMATE VECTOR______________

        State                   unit        index
postion(xyz)                    m           1:3
velocity(xyz)                   m/s         4:6
orientation(quaternion)                     7:10



___________COMMANDS VECTOR_____________________

        State                   unit        index
gyroscopes(xyz)                 rad/s       1:3
accelerometers(xyz)             m/s²        4:6


___________SENSORS VECTOR_____________________

        State                   unit       index
beac1_cam1(uv)                  pixel       1:2
beac2_cam1(uv)                  pixel       3:4
beac1_cam2(uv)                  pixel       5:6
beac2_cam2(uv)                  pixel       7:8

___________PARAMS VECTOR_____________________

        State                   unit       index
cam1_pos(xyz)                   m           1:3
cam1_ori(quaternion)                        4:7
cam1_k                          pixel        8
cam2_pos(xyz)                   m           9:11
cam2_ori(quaternion)                        12:15
cam2_k                          pixel        16
dt                              s            17
g                               m/s²         18
led1_pos(xyz)                   m           19:21
led2_pos(xyz)                   m           22:24

*/

#define Vout (*Ekf::Vin)
#define Min (*Ekf::Min)
#define dt (StateEstimate::dt_proprio) // période mesuré d'échantillonnage des capteurs proprio
#define l1p (Dasta::actuators.led1.position)
#define l2p (Dasta::actuators.led2.position)
#define cam1_p (Dasta::sensors.cam1.position)
#define cam1_q (Dasta::sensors.cam1.orientation)
#define cam1_k (Dasta::sensors.cam1.k)
#define cam2_p (Dasta::sensors.cam2.position)
#define cam2_q (Dasta::sensors.cam2.orientation)
#define cam2_k (Dasta::sensors.cam2.k)

#define GYRO_VAR 0.001
#define ACC_VAR 0.01
#define EXT_VAR 1

#define X_DIM 10
#define Z_DIM 8
#define U_DIM 6
data_type gravity = 9.81;

#define INIT_POS_VAR 1e-6
#define INIT_VEL_VAR 1e-6
#define INIT_ATT_VAR 1e-6

void f(const Vector &x, const Vector &u)
{
    /*
   f =

    X1 + C17*X4
    X2 + C17*X5
    X3 + C17*X6
    C17*U4*X7^2 + 2*C17*U6*X7*X9 - 2*C17*U5*X7*X10 + C17*U4*X8^2 + 2*C17*U5*X8*X9 + 2*C17*U6*X8*X10 - C17*U4*X9^2 - C17*U4*X10^2 + X4
    C17*U5*X7^2 - 2*C17*U6*X7*X8 + 2*C17*U4*X7*X10 - C17*U5*X8^2 + 2*C17*U4*X8*X9 + C17*U5*X9^2 + 2*C17*U6*X9*X10 - C17*U5*X10^2 + X5
    C17*U6*X7^2 + 2*C17*U5*X7*X8 - 2*C17*U4*X7*X9 - C17*U6*X8^2 + 2*C17*U4*X8*X10 - C17*U6*X9^2 + 2*C17*U5*X9*X10 + C17*U6*X10^2 + X6 + C17*C18
    X7 - (C17*(U1*X8 + U2*X9 + U3*X10))/2
    X8 + (C17*(U1*X7 - U2*X10 + U3*X9))/2
    X9 + (C17*(U2*X7 + U1*X10 - U3*X8))/2
    X10 + (C17*(U2*X8 - U1*X9 + U3*X7))/2

*/

    // intermediate variables
    data_type dax = u.data[3] * dt, day = u.data[4] * dt, daz = u.data[5] * dt;

    // integration of linear velocity
    Vout.data[0] = x.data[0] + x.data[3] * dt;
    Vout.data[1] = x.data[1] + x.data[4] * dt;
    Vout.data[2] = x.data[2] + x.data[5] * dt;

    // integration of linear acceleration
    Vout.data[3] = x.data[3] + (dax * x.data[6] * x.data[6] - 2 * daz * x.data[6] * x.data[8] + 2 * day * x.data[6] * x.data[9] + dax * x.data[7] * x.data[7] + 2 * day * x.data[7] * x.data[8] + 2 * daz * x.data[7] * x.data[9] - dax * x.data[8] * x.data[8] - dax * x.data[9] * x.data[9] + u.data[0]) * dt;
    Vout.data[4] = x.data[4] + (day * x.data[6] * x.data[6] + 2 * daz * x.data[6] * x.data[7] - 2 * dax * x.data[6] * x.data[9] - day * x.data[7] * x.data[7] + 2 * dax * x.data[7] * x.data[8] + day * x.data[8] * x.data[8] + 2 * daz * x.data[8] * x.data[9] - day * x.data[9] * x.data[9] + u.data[1]) * dt;
    Vout.data[5] = x.data[5] + (daz * x.data[6] * x.data[6] - 2 * day * x.data[6] * x.data[7] + 2 * dax * x.data[6] * x.data[8] - daz * x.data[7] * x.data[7] + 2 * dax * x.data[7] * x.data[9] - daz * x.data[8] * x.data[8] + 2 * day * x.data[8] * x.data[9] + daz * x.data[9] * x.data[9] + u.data[2] + gravity) * dt;

    // integration of quaternion
    Vout.data[6] = x.data[6] - (dt * (u.data[0] * x.data[7] + u.data[1] * x.data[8] + u.data[2] * x.data[9])) / 2;
    Vout.data[7] = x.data[7] + (dt * (u.data[0] * x.data[6] - u.data[1] * x.data[9] + u.data[2] * x.data[8])) / 2;
    Vout.data[8] = x.data[8] + (dt * (u.data[1] * x.data[6] + u.data[0] * x.data[9] - u.data[2] * x.data[6])) / 2;
    Vout.data[9] = x.data[9] + (dt * (u.data[1] * x.data[7] - u.data[0] * x.data[8] + u.data[2] * x.data[6])) / 2;

    // normalize quaternion
    data_type norm_inv = 1 / sqrt(Vout.data[6] * Vout.data[6] + Vout.data[7] * Vout.data[7] + Vout.data[8] * Vout.data[8] + Vout.data[9] * Vout.data[9]);
    Vout.data[6] *= norm_inv;
    Vout.data[7] *= norm_inv;
    Vout.data[8] *= norm_inv;
    Vout.data[9] *= norm_inv;
}

// fonction de mesure
void h(const Vector &x)
{
    /*
    function Z_predict = measurementFunction(X,C)
        %fonction pour prédire les donnés capteurs exteroceptifs
        % perspective projection for led1
        %function to predict exteroceptive sensor data
        %Z_predict  : l1c1x, l1c1y, l2c1x, l2c1y, l1c2x, l1c2y, l2c2x, l2c2y
        [x,y,z,qw,qx,qy,qz] = deal(X(1),X(2),X(3),X(7),X(8),X(9),X(10));


        %sensors settings
        cam1_p = C(1:3);
        cam1_q = C(4:7);
        cam1_k = C(8);
        cam2_p = C(9:11);
        cam2_q = C(12:15);
        cam2_k = C(16);


        ir1_p = C(17:19);%[0.1 0 0]; % position of led1
        ir2_p = C(20:22);%[-0.1 0 0]; % position of led2

        [l1x,l1y,l1z] = rotate(ir1_p(1),ir1_p(2),ir1_p(3),qw,-qx,-qy,-qz);
        l1x= l1x+x;
        l1y= l1y+y;
        l1z= l1z+z;
        [l1x,l1y,l1z] = rotate(l1x-cam1_p(1),l1y-cam1_p(2),l1z-cam1_p(3),cam1_q(1),-cam1_q(2),-cam1_q(3),-cam1_q(4));
        l1c1 = [l1x,l1y]*cam1_k/l1z;
        [l2x,l2y,l2z] = rotate(ir2_p(1),ir2_p(2),ir2_p(3),qw,-qx,-qy,-qz);
        [l2x,l2y,l2z] = rotate(l2x+x-cam1_p(1),l2y+y-cam1_p(2),l2z+z-cam1_p(3),cam1_q(1),-cam1_q(2),-cam1_q(3),-cam1_q(4));
        l2c1 = [l2x,l2y]*cam1_k/(l2z);

        [l1x,l1y,l1z] = rotate(ir1_p(1),ir1_p(2),ir1_p(3),qw,-qx,-qy,-qz);
        [l1x,l1y,l1z] = rotate(l1x+x-cam2_p(1),l1y+y-cam2_p(2),l1z+z-cam2_p(3),cam2_q(1),-cam2_q(2),-cam2_q(3),-cam2_q(4));
        l1c2 = [l1x,l1y]*cam2_k/(l1z);
        [l2x,l2y,l2z] = rotate(ir2_p(1),ir2_p(2),ir2_p(3),qw,-qx,-qy,-qz);
        [l2x,l2y,l2z] = rotate(l2x+x-cam2_p(1),l2y+y-cam2_p(2),l2z+z-cam2_p(3),cam2_q(1),-cam2_q(2),-cam2_q(3),-cam2_q(4));
        l2c2 = [l2x,l2y]*cam2_k/(l2z);

        Z_predict = [l1c1(1); l1c1(2); l2c1(1); l2c1(2); l1c2(1); l1c2(2); l2c2(1); l2c2(2)];
    end
 */

    Vector l1c1, l2c1, l1c2, l2c2; // to store results (the position of a led in the image)
    l1c1.size = 2;
    l1c1.data = Vout.data;
    l2c1.size = 2;
    l2c1.data = Vout.data + 2;
    l1c2.size = 2;
    l1c2.data = Vout.data + 4;
    l2c2.size = 2;
    l2c2.data = Vout.data + 6;

    Quaternion q;
    q.data = x.data + 6;

    Vector p;
    p.size = 3;
    p.data = x.data;

    Vector l; // to compute the position of a led in the world frame using ekf->tmp1 as temporary storage
    l.size = 3;
    l.data = Ekf::tmp1->data;

    rotate(l, q, l1p);
    add(l, l, p); // position of led1 in world frame

    Vector lc; // to compute the position of a led in a camera frame using ekf->tmp1 as temporary storage
    lc.size = 3;
    lc.data = Ekf::tmp1->data + 3;

    cam1_q.conjugate();
    cam2_q.conjugate();

    sub(lc, l, cam1_p); // position of led1 in cam1 frame
    rotate(lc, cam1_q, lc);
    l1c1.data[0] = lc.data[0] * cam1_k / lc.data[2];
    l1c1.data[1] = lc.data[1] * cam1_k / lc.data[2];

    sub(lc, l, cam2_p); // position of led1 in cam2 frame
    rotate(lc, cam2_q, lc);
    l1c2.data[0] = lc.data[0] * cam2_k / lc.data[2];
    l1c2.data[1] = lc.data[1] * cam2_k / lc.data[2];

    rotate(l, q, l2p); // position of led2 in world frame

    sub(lc, l, cam1_p); // position of led2 in cam1 frame
    rotate(lc, cam1_q, lc);
    l2c1.data[0] = lc.data[0] * cam1_k / lc.data[2];
    l2c1.data[1] = lc.data[1] * cam1_k / lc.data[2];

    sub(lc, l, cam2_p); // position of led2 in cam2 frame
    rotate(lc, cam2_q, lc);
    l2c2.data[0] = lc.data[0] * cam2_k / lc.data[2];

    cam1_q.conjugate();
    cam2_q.conjugate();
}

// jacobienne de f par rapport à x
void Fx(const Vector &x, const Vector &u)
{
    /*
    without normalization of the quaternion at the end of the function (negligible error)
    Jx =

    [1, 0, 0, C17,   0,   0,                              0,                              0,                               0,                               0]
    [0, 1, 0,   0, C17,   0,                              0,                              0,                               0,                               0]
    [0, 0, 1,   0,   0, C17,                              0,                              0,                               0,                               0]
    [0, 0, 0,   1,   0,   0, 2*C17*(U4*X7 - U5*X10 + U6*X9), 2*C17*(U4*X8 + U5*X9 + U6*X10),   2*C17*(U5*X8 - U4*X9 + U6*X7), -2*C17*(U5*X7 + U4*X10 - U6*X8)]
    [0, 0, 0,   0,   1,   0, 2*C17*(U5*X7 + U4*X10 - U6*X8), -2*C17*(U5*X8 - U4*X9 + U6*X7),  2*C17*(U4*X8 + U5*X9 + U6*X10),  2*C17*(U4*X7 - U5*X10 + U6*X9)]
    [0, 0, 0,   0,   0,   1,  2*C17*(U5*X8 - U4*X9 + U6*X7), 2*C17*(U5*X7 + U4*X10 - U6*X8), -2*C17*(U4*X7 - U5*X10 + U6*X9),  2*C17*(U4*X8 + U5*X9 + U6*X10)]
    [0, 0, 0,   0,   0,   0,                              1,                    -(C17*U1)/2,                     -(C17*U2)/2,                     -(C17*U3)/2]
    [0, 0, 0,   0,   0,   0,                     (C17*U1)/2,                              1,                      (C17*U3)/2,                     -(C17*U2)/2]
    [0, 0, 0,   0,   0,   0,                     (C17*U2)/2,                    -(C17*U3)/2,                               1,                      (C17*U1)/2]
    [0, 0, 0,   0,   0,   0,                     (C17*U3)/2,                     (C17*U2)/2,                     -(C17*U1)/2,                               1]

    */

    // intermediate variables
    data_type dvx_dqw = 2 * dt * (u.data[3] * x.data[6] - u.data[4] * x.data[9] + u.data[5] * x.data[8]);
    data_type dvx_dqx = 2 * dt * (u.data[3] * x.data[7] + u.data[4] * x.data[8] + u.data[5] * x.data[9]);
    data_type dvx_dqy = 2 * dt * (u.data[4] * x.data[7] - u.data[3] * x.data[8] + u.data[5] * x.data[6]);
    data_type dvx_dqz = -2 * dt * (u.data[4] * x.data[6] + u.data[3] * x.data[9] - u.data[5] * x.data[7]);

    data_type dgx_h = 0.5 * dt * u.data[0];
    data_type dgy_h = 0.5 * dt * u.data[1];
    data_type dgz_h = 0.5 * dt * u.data[2];

    Min.cols = X_DIM;
    Min.rows = X_DIM;
    Min.set_eye();

    Min.data[3] = dt;
    Min.data[Min.cols + 4] = dt;
    Min.data[2 * Min.cols + 5] = dt;

    Min.data[3 * Min.cols + 6] = dvx_dqw;
    Min.data[3 * Min.cols + 7] = dvx_dqx;
    Min.data[3 * Min.cols + 8] = dvx_dqy;
    Min.data[3 * Min.cols + 9] = dvx_dqz;

    Min.data[4 * Min.cols + 6] = -dvx_dqx;
    Min.data[4 * Min.cols + 7] = dvx_dqw;
    Min.data[4 * Min.cols + 8] = -dvx_dqz;
    Min.data[4 * Min.cols + 9] = dvx_dqy;

    Min.data[5 * Min.cols + 6] = -dvx_dqy;
    Min.data[5 * Min.cols + 7] = dvx_dqz;
    Min.data[5 * Min.cols + 8] = dvx_dqw;
    Min.data[5 * Min.cols + 9] = -dvx_dqx;

    Min.data[6 * Min.cols + 7] = -dgx_h;
    Min.data[6 * Min.cols + 8] = -dgy_h;
    Min.data[6 * Min.cols + 9] = -dgz_h;

    Min.data[7 * Min.cols + 6] = dgx_h;
    Min.data[7 * Min.cols + 8] = dgz_h;
    Min.data[7 * Min.cols + 9] = -dgy_h;

    Min.data[8 * Min.cols + 6] = dgy_h;
    Min.data[8 * Min.cols + 7] = -dgz_h;
    Min.data[8 * Min.cols + 9] = dgx_h;

    Min.data[9 * Min.cols + 6] = dgz_h;
    Min.data[9 * Min.cols + 7] = dgy_h;
    Min.data[9 * Min.cols + 8] = -dgx_h;
}

// jacobienne de f par rapport à u
void Fu(const Vector &x, const Vector &u)
{
    /*
    Ju =

    [          0,            0,            0,                                0,                                0,                                0]
    [          0,            0,            0,                                0,                                0,                                0]
    [          0,            0,            0,                                0,                                0,                                0]
    [          0,            0,            0, C17*(X7^2 + X8^2 - X9^2 - X10^2),          -2*C17*(X7*X10 - X8*X9),           2*C17*(X7*X9 + X8*X10)]
    [          0,            0,            0,           2*C17*(X7*X10 + X8*X9), C17*(X7^2 - X8^2 + X9^2 - X10^2),          -2*C17*(X7*X8 - X9*X10)]
    [          0,            0,            0,          -2*C17*(X7*X9 - X8*X10),           2*C17*(X7*X8 + X9*X10), C17*(X7^2 - X8^2 - X9^2 + X10^2)]
    [-(C17*X8)/2,  -(C17*X9)/2, -(C17*X10)/2,                                0,                                0,                                0]
    [ (C17*X7)/2, -(C17*X10)/2,   (C17*X9)/2,                                0,                                0,                                0]
    [(C17*X10)/2,   (C17*X7)/2,  -(C17*X8)/2,                                0,                                0,                                0]
    [-(C17*X9)/2,   (C17*X8)/2,   (C17*X7)/2,                                0,                                0,                                0]

    */

    Min.cols = X_DIM;
    Min.rows = U_DIM;
    Min.fill(0);

    Min.data[3 * Min.cols + 3] = dt * (x.data[6] * x.data[6] + x.data[7] * x.data[7] - x.data[8] * x.data[8] - x.data[9] * x.data[9]);
    Min.data[3 * Min.cols + 4] = -2 * dt * (x.data[6] * x.data[9] - x.data[7] * x.data[8]);
    Min.data[3 * Min.cols + 5] = 2 * dt * (x.data[6] * x.data[8] + x.data[7] * x.data[9]);

    Min.data[4 * Min.cols + 3] = 2 * dt * (x.data[6] * x.data[9] + x.data[7] * x.data[8]);
    Min.data[4 * Min.cols + 4] = dt * (x.data[6] * x.data[6] - x.data[7] * x.data[7] + x.data[8] * x.data[8] - x.data[9] * x.data[9]);
    Min.data[4 * Min.cols + 5] = -2 * dt * (x.data[6] * x.data[8] - x.data[7] * x.data[9]);

    Min.data[5 * Min.cols + 3] = -2 * dt * (x.data[6] * x.data[8] - x.data[7] * x.data[9]);
    Min.data[5 * Min.cols + 4] = 2 * dt * (x.data[6] * x.data[8] + x.data[7] * x.data[9]);
    Min.data[5 * Min.cols + 5] = dt * (x.data[6] * x.data[6] - x.data[7] * x.data[7] - x.data[8] * x.data[8] + x.data[9] * x.data[9]);

    Min.data[6 * Min.cols + 0] = -0.5 * dt * x.data[8];
    Min.data[6 * Min.cols + 1] = -0.5 * dt * x.data[9];
    Min.data[6 * Min.cols + 2] = -0.5 * dt * x.data[10];

    Min.data[7 * Min.cols + 0] = 0.5 * dt * x.data[7];
    Min.data[7 * Min.cols + 1] = -0.5 * dt * x.data[10];
    Min.data[7 * Min.cols + 2] = 0.5 * dt * x.data[9];

    Min.data[8 * Min.cols + 0] = 0.5 * dt * x.data[10];
    Min.data[8 * Min.cols + 1] = 0.5 * dt * x.data[7];
    Min.data[8 * Min.cols + 2] = -0.5 * dt * x.data[8];

    Min.data[9 * Min.cols + 0] = -0.5 * dt * x.data[9];
    Min.data[9 * Min.cols + 1] = 0.5 * dt * x.data[8];
    Min.data[9 * Min.cols + 2] = 0.5 * dt * x.data[7];
}

void initEKF(Ekf *ekf)
{
    Matrix_f1 *hh = new Matrix_f1[1];
    hh[0] = *h;
    uint_fast8_t *zz_dim = new uint_fast8_t[1];
    zz_dim[0] = Z_DIM;
    ekf = new Ekf(f, hh, X_DIM, zz_dim, U_DIM, Fx, Fu, nullptr, 1);

    ekf->x->fill(0);
    ekf->x->data[6] = 1; // qw

    ekf->P->fill(0);
    ekf->P->operator()(0, 0) = INIT_POS_VAR;
    ekf->P->operator()(1, 1) = INIT_POS_VAR;
    ekf->P->operator()(2, 2) = INIT_POS_VAR;
    ekf->P->operator()(3, 3) = INIT_VEL_VAR;
    ekf->P->operator()(4, 4) = INIT_VEL_VAR;
    ekf->P->operator()(5, 5) = INIT_VEL_VAR;
    ekf->P->operator()(6, 6) = INIT_ATT_VAR;
    ekf->P->operator()(7, 7) = INIT_ATT_VAR;
    ekf->P->operator()(8, 8) = INIT_ATT_VAR;
    ekf->P->operator()(9, 9) = INIT_ATT_VAR;

    ekf->R[0]->fill(0);
    ekf->R[0]->operator()(0, 0) = EXT_VAR;
    ekf->R[0]->operator()(1, 1) = EXT_VAR;
    ekf->R[0]->operator()(2, 2) = EXT_VAR;

    ekf->Q->fill(0);
    ekf->Q->operator()(0, 0) = GYRO_VAR;
    ekf->Q->operator()(1, 1) = GYRO_VAR;
    ekf->Q->operator()(2, 2) = GYRO_VAR;
    ekf->Q->operator()(3, 3) = ACC_VAR;
    ekf->Q->operator()(4, 4) = ACC_VAR;
    ekf->Q->operator()(5, 5) = ACC_VAR;
}

#endif // STATEESTIMATECONFIG_HPP