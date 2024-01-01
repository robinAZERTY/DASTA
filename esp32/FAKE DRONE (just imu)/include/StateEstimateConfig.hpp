#ifndef STATEESTIMATECONFIG_HPP
#define STATEESTIMATECONFIG_HPP
#include "StateEstimate.hpp"

#define Vout (*Ekf::Vin)
#define Min (*Ekf::Min)
#define dt (StateEstimate::dt_proprio) // période mesuré d'échantillonnage des capteurs proprio

#define X_DIM 10 // X = [x,y,z,vx,vy,vz,qw,qx,qy,qz,]
#define Z_DIM 3 // Z = [c1l1x,c1l1y,c1l2x,c1l2y,c1l3x,c1l3y,c1l4x,c1l4y,c2l1x,c2l1y,c2l2x,c2l2y,c2l3x,c2l3y,c2l4x,c2l4y]
#define U_DIM 6  // U = [gx,gy,gz,ax,ay,az]
#define gravity 9.81

#define GYRO_VAR 0.002
#define ACC_VAR 0.05
#define EXT_VAR 0.1
Vector tmp3(3);
Vector Xcp(X_DIM);
Quaternion tmpq(1, 0, 0, 0);

void f(Vector &x, Vector &u)
{
    cd(Xcp, x);
    Vector pos,pos_out;
    pos.data = Xcp.data;
    pos.size = 3;

    pos_out.data = Vout.data;
    pos_out.size = 3;

    Vector vel, vel_out;
    vel.data = Xcp.data + 3;
    vel.size = 3;
    
    vel_out.data = Vout.data + 3;
    vel_out.size = 3;

    Quaternion q, q_out;
    q.data = Xcp.data + 6;
    q_out.data = Vout.data + 6;

    Vector gyro;
    gyro.data = u.data;
    gyro.size = 3;

    Vector acc;
    acc.data = u.data + 3;
    acc.size = 3;


    rotate(tmp3, q, acc); // tmp3 = q*acc*q^-1
    tmp3(2) -= gravity; // tmp3 = q*acc*q^-1 - gravity
    mul(tmp3, tmp3, dt); // tmp3 = dt*tmp3
    add(vel_out, vel, tmp3); // vel = vel + dt*tmp3
    // vel_out.set_zero();


    mul(tmp3, vel, dt); // tmp3 = dt*vel
    add(pos_out, pos, tmp3); // pos = pos + tmp3
    // pos_out.set_zero();

    tmpq(0) = 0;
    tmpq(1) = gyro(0);
    tmpq(2) = gyro(1);
    tmpq(3) = gyro(2);

    mul(q_out, q, tmpq); //  q_out = q*tmpq
    mul(q_out, q_out, 0.5*dt); //  tmpq = 0.5*q_out
    add(q_out, q, q_out); //  q_out = q + tmpq2 -> q_out = q + 0.5*dt*(q*q_gyro)
    q_out.normalize();
}

// fonction de mesure
void h(Vector &x)
{
    Vout(0) = x(0);
    Vout(1) = x(1);
    Vout(2) = x(2);
}

// jacobienne de f par rapport à x
void Fx(Vector &x, Vector &u)
{
    Vector pos;
    pos.data = x.data;
    pos.size = 3;

    Vector vel;
    vel.data = x.data + 3;
    vel.size = 3;
    
    Quaternion q;
    q.data = x.data + 6;

    Vector gyro;
    gyro.data = u.data;
    gyro.size = 3;

    Vector acc;
    acc.data = u.data + 3;
    acc.size = 3;

    Min.set_eye();
    Min(0, 3) = dt; // dx/dvx
    Min(1, 4) = dt; // dy/dvy
    Min(2, 5) = dt; // dz/dvz

    /*
    in the rotate function, we have:
    d/dq_0((q_0 q_0 + q1 q1 - q2 q2 - q3 q3) v0 + (2 q1 q2 - 2 q_0 q3) v1 + (2 q1 q3 + 2 q_0 q2) v2) = 2 (q_0 v0 + q2 v2 - q3 v1) = 2 (q_0 v0 + q2 v2 - q3 v1)
    d/dq_1((q_0 q_0 + q1 q1 - q2 q2 - q3 q3) v0 + (2 q1 q2 - 2 q_0 q3) v1 + (2 q1 q3 + 2 q_0 q2) v2) = 2 (q_0 v1 + q3 v0 - q1 v2) = 2 (q_0 v1 + q3 v0 - q1 v2)
    d/dq_2((q_0 q_0 + q1 q1 - q2 q2 - q3 q3) v0 + (2 q1 q2 - 2 q_0 q3) v1 + (2 q1 q3 + 2 q_0 q2) v2) = 2 (q_0 v2 + q1 v1 - q2 v0) = 2 (q_0 v2 + q1 v1 - q2 v0)
    d/dq_3((q_0 q_0 + q1 q1 - q2 q2 - q3 q3) v0 + (2 q1 q2 - 2 q_0 q3) v1 + (2 q1 q3 + 2 q_0 q2) v2) = 2 (q_0 v3 + q2 v0 - q1 v0) = 2 (q_0 v3 + q2 v0 - q1 v0)

    d/dq_0((2 q1 q2 + 2 q_0 q3) v0 + (q_0 q_0 - q1 q1 + q2 q2 - q3 q3) v1 + (2 q2 q3 - 2 q_0 q1) v2) = 2 (q_0 v1 - q1 v2 + q3 v0)   
    d/dq_1((2 q1 q2 + 2 q_0 q3) v0 + (q_0 q_0 - q1 q1 + q2 q2 - q3 q3) v1 + (2 q2 q3 - 2 q_0 q1) v2) = 2 (q_0 v2 + q2 v1 - q1 v0)
    d/dq_2((2 q1 q2 + 2 q_0 q3) v0 + (q_0 q_0 - q_1 q_1 + q_2 q_2 - q_3 q_3) v1 + (2 q_2 q_3 - 2 q_0 q_1) v2) = 2 (q_0 v3 - q_3 v0 + q_2 v1)
    d/dq_3((2 q1 q2 + 2 q_0 q3) v0 + (q_0 q_0 - q_1 q_1 + q_2 q_2 - q_3 q_3) v1 + (2 q_2 q_3 - 2 q_0 q_1) v2) = 2 (q_0 v0 + q_1 v1 - q_2 v2)

    d/dq_0((2 q1 q3 - 2 q_0 q2) v0 + (2 q2 q3 + 2 q_0 q1) v1 + (q_0 q_0 - q1 q1 - q2 q2 + q3 q3) v2) = 2 (q_0 v2 + q1 v1 - q2 v0)
    d/dq_1((2 q1 q3 - 2 q_0 q2) v0 + (2 q2 q3 + 2 q_0 q1) v1 + (q_0 q_0 - q1 q1 - q2 q2 + q3 q3) v2) = 2 (q_0 v3 - q3 v0 + q2 v1)
    d/dq_2((2 q1 q3 - 2 q_0 q2) v0 + (2 q2 q3 + 2 q_0 q1) v1 + (q_0 q_0 - q_1 q_1 - q_2 q_2 + q_3 q_3) v2) = 2 (q_0 v0 + q_1 v1 - q_2 v2)
    d/dq_3((2 q1 q3 - 2 q_0 q2) v0 + (2 q2 q3 + 2 q_0 q1) v1 + (q_0 q_0 - q_1 q_1 - q_2 q_2 + q_3 q_3) v2) = 2 (q_0 v1 + q_2 v0 - q_3 v0) 
    
    */
    Min(3, 6) = 2 *(q(0)*acc(0) + q(2)*acc(2) - q(3)*acc(1)) * dt; // dvx/dqw
    Min(3, 7) = 2 *(q(0)*acc(1) + q(3)*acc(0) - q(1)*acc(2)) * dt; // dvx/dqx
    Min(3, 8) = 2 *(q(0)*acc(2) + q(1)*acc(1) - q(2)*acc(0)) * dt; // dvx/dqy
    Min(3, 9) = 2 *(q(0)*acc(3) + q(2)*acc(0) - q(1)*acc(1)) * dt; // dvx/dqz

    Min(4, 6) = 2 *(q(0)*acc(1) + q(3)*acc(0) - q(1)*acc(2)) * dt; // dvy/dqw
    Min(4, 7) = 2 *(q(0)*acc(0) + q(2)*acc(2) - q(3)*acc(1)) * dt; // dvy/dqx
    Min(4, 8) = 2 *(q(0)*acc(3) + q(2)*acc(0) - q(1)*acc(1)) * dt; // dvy/dqy
    Min(4, 9) = 2 *(q(0)*acc(2) + q(1)*acc(1) - q(2)*acc(0)) * dt; // dvy/dqz

    Min(5, 6) = 2 *(q(0)*acc(2) + q(1)*acc(1) - q(2)*acc(0)) * dt; // dvz/dqw
    Min(5, 7) = 2 *(q(0)*acc(3) + q(2)*acc(0) - q(1)*acc(1)) * dt; // dvz/dqx
    Min(5, 8) = 2 *(q(0)*acc(0) + q(2)*acc(2) - q(3)*acc(1)) * dt; // dvz/dqy
    Min(5, 9) = 2 *(q(0)*acc(1) + q(3)*acc(0) - q(1)*acc(2)) * dt; // dvz/dqz

    /*
    in the mul(q1, q2) function, we have:
    d/dq1_0(q1_0 q2_0 - q1_1 q2_1 - q1_2 q2_2 - q1_3 q2_3) = q2_0
    d/dq1_1(q1_0 q2_0 - q1_1 q2_1 - q1_2 q2_2 - q1_3 q2_3) = -q2_1
    d/dq1_2(q1_0 q2_0 - q1_1 q2_1 - q1_2 q2_2 - q1_3 q2_3) = -q2_2
    d/dq1_3(q1_0 q2_0 - q1_1 q2_1 - q1_2 q2_2 - q1_3 q2_3) = -q2_3

    d/dq1_0(q1_0 q2_1 + q1_1 q2_0 + q1_2 q2_3 - q1_3 q2_2) = q2_1
    d/dq1_1(q1_0 q2_1 + q1_1 q2_0 + q1_2 q2_3 - q1_3 q2_2) = q2_0
    d/dq1_2(q1_0 q2_1 + q1_1 q2_0 + q1_2 q2_3 - q1_3 q2_2) = q2_3
    d/dq1_3(q1_0 q2_1 + q1_1 q2_0 + q1_2 q2_3 - q1_3 q2_2) = -q2_2

    d/dq1_0(q1_0 q2_2 - q1_1 q2_3 + q1_2 q2_0 + q1_3 q2_1) = q2_2
    d/dq1_1(q1_0 q2_2 - q1_1 q2_3 + q1_2 q2_0 + q1_3 q2_1) = -q2_3
    d/dq1_2(q1_0 q2_2 - q1_1 q2_3 + q1_2 q2_0 + q1_3 q2_1) = q2_0
    d/dq1_3(q1_0 q2_2 - q1_1 q2_3 + q1_2 q2_0 + q1_3 q2_1) = q2_1

    d/dq1_0(q1_0 q2_3 + q1_1 q2_2 - q1_2 q2_1 + q1_3 q2_0) = q2_3
    d/dq1_1(q1_0 q2_3 + q1_1 q2_2 - q1_2 q2_1 + q1_3 q2_0) = q2_2
    d/dq1_2(q1_0 q2_3 + q1_1 q2_2 - q1_2 q2_1 + q1_3 q2_0) = -q2_1
    d/dq1_3(q1_0 q2_3 + q1_1 q2_2 - q1_2 q2_1 + q1_3 q2_0) = q2_0

    and in f, q2_0 = 0
    */

    // Min(6, 6) = 1; // dqw/dqw
    Min(6, 7) = -0.5*dt*gyro(0); // dqw/dqx
    Min(6, 8) = -0.5*dt*gyro(1); // dqw/dqy
    Min(6, 9) = -0.5*dt*gyro(2); // dqw/dqz

    Min(7, 6) = 0.5*dt*gyro(0); // dqx/dqw
    // Min(7, 7) = 1; // dqx/dqx
    Min(7, 8) = 0.5*dt*gyro(2); // dqx/dqy
    Min(7, 9) = -0.5*dt*gyro(1); // dqx/dqz

    Min(8, 6) = 0.5*dt*gyro(1); // dqy/dqw
    Min(8, 7) = -0.5*dt*gyro(2); // dqy/dqx
    // Min(8, 8) = 1; // dqy/dqy
    Min(8, 9) = 0.5*dt*gyro(0); // dqy/dqz

    Min(9, 6) = 0.5*dt*gyro(2); // dqz/dqw
    Min(9, 7) = 0.5*dt*gyro(1); // dqz/dqx
    Min(9, 8) = -0.5*dt*gyro(0); // dqz/dqy
    // Min(9, 9) = 1; // dqz/dqz

}

// jacobienne de f par rapport à u
void Fu(Vector &x, Vector &u)
{
    Vector pos;
    pos.data = x.data;
    pos.size = 3;

    Vector vel;
    vel.data = x.data + 3;
    vel.size = 3;
    
    Quaternion q;
    q.data = x.data + 6;

    Vector gyro;
    gyro.data = u.data;
    gyro.size = 3;

    Vector acc;
    acc.data = u.data + 3;
    acc.size = 3;

    Min.set_zero();

     /*
    in the rotate function, we have:
    d/v0((q_0 q_0 + q1 q1 - q2 q2 - q3 q3) v0 + (2 q1 q2 - 2 q_0 q3) v1 + (2 q1 q3 + 2 q_0 q2) v2) = q_0 q_0 + q1 q1 - q2 q2 - q3 q3
    d/v1((q_0 q_0 + q1 q1 - q2 q2 - q3 q3) v0 + (2 q1 q2 - 2 q_0 q3) v1 + (2 q1 q3 + 2 q_0 q2) v2) = 2 q1 q2 - 2 q_0 q3
    d/v2((q_0 q_0 + q1 q1 - q2 q2 - q3 q3) v0 + (2 q1 q2 - 2 q_0 q3) v1 + (2 q1 q3 + 2 q_0 q2) v2) = 2 q1 q3 + 2 q_0 q2

    d/v0((2 q1 q2 + 2 q_0 q3) v0 + (q_0 q_0 - q1 q1 + q2 q2 - q3 q3) v1 + (2 q2 q3 - 2 q_0 q1) v2) = 2 q_0 q1 - 2 q3 q2
    d/v1((2 q1 q2 + 2 q_0 q3) v0 + (q_0 q_0 - q1 q1 + q2 q2 - q3 q3) v1 + (2 q2 q3 - 2 q_0 q1) v2) = q_0 q_0 - q1 q1 + q2 q2 - q3 q3
    d/v2((2 q1 q2 + 2 q_0 q3) v0 + (q_0 q_0 - q_1 q_1 + q_2 q_2 - q_3 q_3) v1 + (2 q_2 q_3 - 2 q_0 q_1) v2) = 2 q2 q3 - 2 q_0 q1

    d/v0((2 q1 q3 - 2 q_0 q2) v0 + (2 q2 q3 + 2 q_0 q1) v1 + (q_0 q_0 - q1 q1 - q2 q2 + q3 q3) v2) = 2 q_0 q2 + 2 q1 q3
    d/v1((2 q1 q3 - 2 q_0 q2) v0 + (2 q2 q3 + 2 q_0 q1) v1 + (q_0 q_0 - q1 q1 - q2 q2 + q3 q3) v2) = 2 q_0 q1 - 2 q2 q3
    d/v2((2 q1 q3 - 2 q_0 q2) v0 + (2 q2 q3 + 2 q_0 q1) v1 + (q_0 q_0 - q_1 q_1 - q_2 q_2 + q_3 q_3) v2) = q_0 q_0 - q_1 q_1 - q_2 q_2 + q_3 q_3
    
    and in f, we rotate acc by q
    */
    Min(3,3) = dt * (q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3)); // dvx/dax
    Min(3,4) = dt * 2 *(q(1)*q(2) - q(0)*q(3)); // dvx/day
    Min(3,5) = dt * 2 *(q(1)*q(3) + q(0)*q(2)); // dvx/daz

    Min(4,3) = dt * 2 *(q(1)*q(2) + q(0)*q(3)); // dvy/dax
    Min(4,4) = dt * (q(0)*q(0) - q(1)*q(1) + q(2)*q(2) - q(3)*q(3)); // dvy/day
    Min(4,5) = dt * 2 *(q(2)*q(3) - q(0)*q(1)); // dvy/daz

    Min(5,3) = dt * 2 *(q(1)*q(3) - q(0)*q(2)); // dvz/dax
    Min(5,4) = dt * 2 *(q(2)*q(3) + q(0)*q(1)); // dvz/day
    Min(5,5) = dt * (q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3)); // dvz/daz




    /*
        in the mul(q1, q2) function, we have:
    d/dq2_0(q1_0 q2_0 - q1_1 q2_1 - q1_2 q2_2 - q1_3 q2_3) = q1_0
    d/dq2_1(q1_0 q2_0 - q1_1 q2_1 - q1_2 q2_2 - q1_3 q2_3) = -q1_1
    d/dq2_2(q1_0 q2_0 - q1_1 q2_1 - q1_2 q2_2 - q1_3 q2_3) = -q1_2
    d/dq2_3(q1_0 q2_0 - q1_1 q2_1 - q1_2 q2_2 - q1_3 q2_3) = -q1_3

    d/dq2_0(q1_0 q2_1 + q1_1 q2_0 + q1_2 q2_3 - q1_3 q2_2) = q1_1
    d/dq2_1(q1_0 q2_1 + q1_1 q2_0 + q1_2 q2_3 - q1_3 q2_2) = q1_0
    d/dq2_2(q1_0 q2_1 + q1_1 q2_0 + q1_2 q2_3 - q1_3 q2_2) = q1_3
    d/dq2_3(q1_0 q2_1 + q1_1 q2_0 + q1_2 q2_3 - q1_3 q2_2) = -q1_2

    d/dq2_0(q1_0 q2_2 - q1_1 q2_3 + q1_2 q2_0 + q1_3 q2_1) = q1_2
    d/dq2_1(q1_0 q2_2 - q1_1 q2_3 + q1_2 q2_0 + q1_3 q2_1) = -q1_3
    d/dq2_2(q1_0 q2_2 - q1_1 q2_3 + q1_2 q2_0 + q1_3 q2_1) = q1_0
    d/dq2_3(q1_0 q2_2 - q1_1 q2_3 + q1_2 q2_0 + q1_3 q2_1) = q1_1

    d/dq2_0(q1_0 q2_3 + q1_1 q2_2 - q1_2 q2_1 + q1_3 q2_0) = q1_3
    d/dq2_1(q1_0 q2_3 + q1_1 q2_2 - q1_2 q2_1 + q1_3 q2_0) = q1_2
    d/dq2_2(q1_0 q2_3 + q1_1 q2_2 - q1_2 q2_1 + q1_3 q2_0) = -q1_1
    d/dq2_3(q1_0 q2_3 + q1_1 q2_2 - q1_2 q2_1 + q1_3 q2_0) = q1_0

    but in f, q2_0 = 0

    */
    Min(6,0) = -0.5*dt*q(1); // dqw/dgx
    Min(6,1) = -0.5*dt*q(2); // dqw/dgy
    Min(6,2) = -0.5*dt*q(3); // dqw/dgz

    Min(7,0) = 0.5*dt*q(0); // dqx/dgx
    Min(7,1) = -0.5*dt*q(3); // dqx/dgy
    Min(7,2) = 0.5*dt*q(2); // dqx/dgz

    Min(8,0) = 0.5*dt*q(3); // dqy/dgx
    Min(8,1) = 0.5*dt*q(0); // dqy/dgy
    Min(8,2) = -0.5*dt*q(1); // dqy/dgz

    Min(9,0) = -0.5*dt*q(2); // dqz/dgx
    Min(9,1) = 0.5*dt*q(1); // dqz/dgy
    Min(9,2) = 0.5*dt*q(0); // dqz/dgz
}

// jacobienne de h par rapport à x
void H(Vector &x)
{
    Min.set_eye();
}

#endif // STATEESTIMATECONFIG_HPP