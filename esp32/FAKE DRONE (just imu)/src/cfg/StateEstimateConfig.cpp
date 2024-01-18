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

#define X_DIM 10
#define Z_DIM 8
#define U_DIM 6
#define C_DIM 24

#define Vout (*Ekf::Vin)
#define Min (*Ekf::Min)

#define GYRO_VAR 0.001
#define ACC_VAR 0.01
#define EXT_VAR 1


#define INITIAL_GRAVITY 9.81

#define INIT_POS_VAR 1e-6
#define INIT_VEL_VAR 1e-6
#define INIT_ATT_VAR 1e-6

void f(const Vector &x, const Vector &u, const Vector &c)
{
    data_type dt = c.data[16];
    data_type g = c.data[17];
    
    // intermediate variables
    data_type dax = u.data[3] * dt, day = u.data[4] * dt, daz = u.data[5] * dt;

    // integration of linear velocity
    Vout.data[0] = x.data[0] + x.data[3] * dt;
    Vout.data[1] = x.data[1] + x.data[4] * dt;
    Vout.data[2] = x.data[2] + x.data[5] * dt;

    // integration of linear acceleration
    Vout.data[3] = x.data[3] + (dax * x.data[6] * x.data[6] - 2 * daz * x.data[6] * x.data[8] + 2 * day * x.data[6] * x.data[9] + dax * x.data[7] * x.data[7] + 2 * day * x.data[7] * x.data[8] + 2 * daz * x.data[7] * x.data[9] - dax * x.data[8] * x.data[8] - dax * x.data[9] * x.data[9] + u.data[0]) * dt;
    Vout.data[4] = x.data[4] + (day * x.data[6] * x.data[6] + 2 * daz * x.data[6] * x.data[7] - 2 * dax * x.data[6] * x.data[9] - day * x.data[7] * x.data[7] + 2 * dax * x.data[7] * x.data[8] + day * x.data[8] * x.data[8] + 2 * daz * x.data[8] * x.data[9] - day * x.data[9] * x.data[9] + u.data[1]) * dt;
    Vout.data[5] = x.data[5] + (daz * x.data[6] * x.data[6] - 2 * day * x.data[6] * x.data[7] + 2 * dax * x.data[6] * x.data[8] - daz * x.data[7] * x.data[7] + 2 * dax * x.data[7] * x.data[9] - daz * x.data[8] * x.data[8] + 2 * day * x.data[8] * x.data[9] + daz * x.data[9] * x.data[9] + u.data[2] + g) * dt;

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
void h(const Vector &x, const Vector &c)
{
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

    Vector cam1_p, cam2_p; // position of the cameras in the world frame
    cam1_p.size = 3;
    cam1_p.data = c.data;
    cam2_p.size = 3;
    cam2_p.data = c.data + 9;

    Quaternion cam1_q, cam2_q; // orientation of the cameras in the world frame
    cam1_q.data = c.data + 3;
    cam2_q.data = c.data + 12;

    data_type cam1_k = c.data[8];
    data_type cam2_k = c.data[17];

    Vector l1p, l2p; // position of the leds in the quad frame
    l1p.size = 3;
    l1p.data = c.data + 18;
    l2p.size = 3;
    l2p.data = c.data + 21;

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
void Fx(const Vector &x, const Vector &u, const Vector &c)
{
    data_type dt = c.data[16];

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
void Fu(const Vector &x, const Vector &u, const Vector &c)
{
    data_type dt = c.data[16];
    
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

void Dasta::configureStateEstimate()
{
    Matrix_f1 *hh = new Matrix_f1[1];
    hh[0] = *h;
    uint_fast8_t *zz_dim = new uint_fast8_t[1];
    zz_dim[0] = Z_DIM;
    estimator.ekf = new Ekf(f, hh, X_DIM, zz_dim, U_DIM, Fx, Fu, nullptr, 1);

    estimator.ekf->x->fill(0);
    estimator.ekf->x->data[6] = 1; // qw

    estimator.ekf->P->fill(0);
    estimator.ekf->P->operator()(0, 0) = INIT_POS_VAR;
    estimator.ekf->P->operator()(1, 1) = INIT_POS_VAR;
    estimator.ekf->P->operator()(2, 2) = INIT_POS_VAR;
    estimator.ekf->P->operator()(3, 3) = INIT_VEL_VAR;
    estimator.ekf->P->operator()(4, 4) = INIT_VEL_VAR;
    estimator.ekf->P->operator()(5, 5) = INIT_VEL_VAR;
    estimator.ekf->P->operator()(6, 6) = INIT_ATT_VAR;
    estimator.ekf->P->operator()(7, 7) = INIT_ATT_VAR;
    estimator.ekf->P->operator()(8, 8) = INIT_ATT_VAR;
    estimator.ekf->P->operator()(9, 9) = INIT_ATT_VAR;

    estimator.ekf->R[0]->fill(0);
    estimator.ekf->R[0]->operator()(0, 0) = EXT_VAR;
    estimator.ekf->R[0]->operator()(1, 1) = EXT_VAR;
    estimator.ekf->R[0]->operator()(2, 2) = EXT_VAR;

    estimator.ekf->Q->fill(0);
    estimator.ekf->Q->operator()(0, 0) = GYRO_VAR;
    estimator.ekf->Q->operator()(1, 1) = GYRO_VAR;
    estimator.ekf->Q->operator()(2, 2) = GYRO_VAR;
    estimator.ekf->Q->operator()(3, 3) = ACC_VAR;
    estimator.ekf->Q->operator()(4, 4) = ACC_VAR;
    estimator.ekf->Q->operator()(5, 5) = ACC_VAR;

    estimator.gravity = INITIAL_GRAVITY;

    // link the vectors
    
    estimator.position.data = estimator.ekf->x->data;
    estimator.velocity.data = estimator.ekf->x->data + 3;
    estimator.orientation.data = estimator.ekf->x->data + 6;
}