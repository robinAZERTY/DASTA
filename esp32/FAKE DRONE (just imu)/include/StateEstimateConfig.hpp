#ifndef STATEESTIMATECONFIG_HPP
#define STATEESTIMATECONFIG_HPP
#include "StateEstimate.hpp"

#define Vout (*Ekf::Vin)
#define Min (*Ekf::Min)
#define dt (StateEstimate::dt_proprio) // période mesuré d'échantillonnage des capteurs proprio

// #define GYRO_VAR 0.001
// #define ACC_VAR 4
// #define EXT_VAR 0.1

// #define X_DIM 10 // X = [x,y,z,vx,vy,vz,qw,qx,qy,qz]
// #define Z_DIM 3  // Z = [x,y,z]
// #define U_DIM 6  // U = [gx,gy,gz,ax,ay,az]
// #define gravity -9.81

// void f(Vector &x, Vector &u)
// {
//     /*
//     function next_X = transitionFunction(prev_X, U, params)
//         %X_est = [x y z vx vy vz qw qx qy qz] -> position, orientation and linear velocity
//         %U = [gx gy gz ax ay az] ->
//         %function for state transition (pour la prediction using commands
//         %or proprioceptive sensors)

//         [x, y, z, vx, vy, vz, qw, qx, qy, qz] = deal(prev_X(1),prev_X(2),prev_X(3),prev_X(4), ...
//                                 prev_X(5),prev_X(6),prev_X(7),prev_X(8),prev_X(9),prev_X(10));
//         [gx,gy,gz,ax,ay,az] = deal(U(1),U(2),U(3),U(4),U(5),U(6));

//         dt=params(1);
//         gravity = params(2);

//         x_ = x+vx*dt;
//         y_ = y+vy*dt;
//         z_ = z+vz*dt;					% integration of linear velocity

//         ac_xyz = [ax;ay;az];
//         [ae_x,ae_y,ae_z] = rotate(ac_xyz(1),ac_xyz(2),ac_xyz(3),qw,-qx,-qy,-qz);
//         vx_ = vx+ae_x*dt;
//         vy_ = vy+ae_y*dt;
//         vz_ = vz+(ae_z-gravity)*dt;				% integration of linear acceleration

//         gc_xyz = [gx;gy;gz];

//         [dqw,dqx,dqy,dqz] = quatMul(qw,qx,qy,qz,0,gc_xyz(1),gc_xyz(2),gc_xyz(3));
//         q_ = [qw,qx,qy,qz] + 0.5*dt*[dqw,dqx,dqy,dqz];

//         next_X = [x_ y_ z_ vx_ vy_ vz_ q_(1) q_(2) q_(3) q_(4)];
//     end

//     transpose(simplify(transitionFunction([x y z vx vy vz qw qx qy qz], [gx gy gz ax ay az],[dt, gravity]))) =
//     x + dt*vx
//     y + dt*vy
//     z + dt*vz
//     ax*dt*qw^2 - 2*az*dt*qw*qy + 2*ay*dt*qw*qz + ax*dt*qx^2 + 2*ay*dt*qx*qy + 2*az*dt*qx*qz - ax*dt*qy^2 - ax*dt*qz^2 + vx
//     ay*dt*qw^2 + 2*az*dt*qw*qx - 2*ax*dt*qw*qz - ay*dt*qx^2 + 2*ax*dt*qx*qy + ay*dt*qy^2 + 2*az*dt*qy*qz - ay*dt*qz^2 + vy
//     az*dt*qw^2 - 2*ay*dt*qw*qx + 2*ax*dt*qw*qy - az*dt*qx^2 + 2*ax*dt*qx*qz - az*dt*qy^2 + 2*ay*dt*qy*qz + az*dt*qz^2 + vz - dt*gravity
//     qw - (dt*(gx*qx + gy*qy + gz*qz))/2
//     qx + (dt*(gx*qw - gy*qz + gz*qy))/2
//     qy + (dt*(gy*qw + gx*qz - gz*qx))/2
//     qz + (dt*(gy*qx - gx*qy + gz*qw))/2

//     Do not forget to add the normalization of the quaternion at the end of the function
// */

// //intermediate variables
// data_type dax = u(3)*dt, day = u(4)*dt, daz = u(5)*dt;

// //integration of linear velocity
// Vout(0) = x(0) + x(3)*dt;
// Vout(1) = x(1) + x(4)*dt;
// Vout(2) = x(2) + x(5)*dt;

// //integration of linear acceleration
// Vout(3) = x(3) + (dax*x(6)*x(6) - 2*daz*x(6)*x(8) + 2*day*x(6)*x(9) + dax*x(7)*x(7) + 2*day*x(7)*x(8) + 2*daz*x(7)*x(9) - dax*x(8)*x(8) - dax*x(9)*x(9) + u(0))*dt;
// Vout(4) = x(4) + (day*x(6)*x(6) + 2*daz*x(6)*x(7) - 2*dax*x(6)*x(9) - day*x(7)*x(7) + 2*dax*x(7)*x(8) + day*x(8)*x(8) + 2*daz*x(8)*x(9) - day*x(9)*x(9) + u(1))*dt;
// Vout(5) = x(5) + (daz*x(6)*x(6) - 2*day*x(6)*x(7) + 2*dax*x(6)*x(8) - daz*x(7)*x(7) + 2*dax*x(7)*x(9) - daz*x(8)*x(8) + 2*day*x(8)*x(9) + daz*x(9)*x(9) + u(2) - gravity)*dt;

// //integration of quaternion
// Vout(6) = x(6) - (dt*(u(0)*x(7) + u(1)*x(8) + u(2)*x(9)))/2;
// Vout(7) = x(7) + (dt*(u(0)*x(6) - u(1)*x(9) + u(2)*x(8)))/2;
// Vout(8) = x(8) + (dt*(u(1)*x(6) + u(0)*x(9) - u(2)*x(6)))/2;
// Vout(9) = x(9) + (dt*(u(1)*x(7) - u(0)*x(8) + u(2)*x(6)))/2;

// //normalize quaternion
// data_type norm_inv = 1/sqrt(Vout(6)*Vout(6) + Vout(7)*Vout(7) + Vout(8)*Vout(8) + Vout(9)*Vout(9));
// Vout(6) *= norm_inv;
// Vout(7) *= norm_inv;
// Vout(8) *= norm_inv;
// Vout(9) *= norm_inv;

// }

// // fonction de mesure
// void h(Vector &x)
// {
//     Vout(0) = x(0);
//     Vout(1) = x(1);
//     Vout(2) = x(2);
// }

// // jacobienne de f par rapport à x
// void Fx(Vector &x, Vector &u)
// {
//     /*
//     without normalization of the quaternion at the end of the function (negligible error)
//     Jx =

//         [1, 0, 0, dt,  0,  0,                                0,                                 0,                                 0,                                 0]
//         [0, 1, 0,  0, dt,  0,                                0,                                 0,                                 0,                                 0]
//         [0, 0, 1,  0,  0, dt,                                0,                                 0,                                 0,                                 0]
//         [0, 0, 0,  1,  0,  0, dt*(2*ax*qw + 2*ay*qz - 2*az*qy),  dt*(2*ax*qx + 2*ay*qy + 2*az*qz), -dt*(2*ax*qy - 2*ay*qx + 2*az*qw),  dt*(2*ay*qw - 2*ax*qz + 2*az*qx)]
//         [0, 0, 0,  0,  1,  0, dt*(2*ay*qw - 2*ax*qz + 2*az*qx),  dt*(2*ax*qy - 2*ay*qx + 2*az*qw),  dt*(2*ax*qx + 2*ay*qy + 2*az*qz), -dt*(2*ax*qw + 2*ay*qz - 2*az*qy)]
//         [0, 0, 0,  0,  0,  1, dt*(2*ax*qy - 2*ay*qx + 2*az*qw), -dt*(2*ay*qw - 2*ax*qz + 2*az*qx),  dt*(2*ax*qw + 2*ay*qz - 2*az*qy),  dt*(2*ax*qx + 2*ay*qy + 2*az*qz)]
//         [0, 0, 0,  0,  0,  0,                                1,                        -(dt*gx)/2,                        -(dt*gy)/2,                        -(dt*gz)/2]
//         [0, 0, 0,  0,  0,  0,                        (dt*gx)/2,                                 1,                         (dt*gz)/2,                        -(dt*gy)/2]
//         [0, 0, 0,  0,  0,  0,                        (dt*gy)/2,                        -(dt*gz)/2,                                 1,                         (dt*gx)/2]
//         [0, 0, 0,  0,  0,  0,                        (dt*gz)/2,                         (dt*gy)/2,                        -(dt*gx)/2,                                 1]
//     */

//     //intermediate variables
//     data_type dvx_dqw = dt*(2*u(3)*x(6) + 2*u(4)*x(9) - 2*u(5)*x(8));
//     data_type dvx_dqx = dt*(2*u(3)*x(7) + 2*u(4)*x(8) + 2*u(5)*x(9));
//     data_type dvx_dqy = -dt*(2*u(3)*x(8) - 2*u(4)*x(7) + 2*u(5)*x(6));
//     data_type dvx_dqz = dt*(-2*u(3)*x(9) + 2*u(4)*x(6) + 2*u(5)*x(7));

//     data_type dgx_h = dt*u(0)/2, dgy_h = dt*u(1)/2, dgz_h = dt*u(2)/2;

//     Min.set_eye();
//     Min(0,3) = dt;
//     Min(1,4) = dt;
//     Min(2,5) = dt;

//     Min(3,6) = dvx_dqw;
//     Min(3,7) = dvx_dqx;
//     Min(3,8) = dvx_dqy;
//     Min(3,9) = dvx_dqz;

//     Min(4,6) = dvx_dqz;
//     Min(4,7) = -dvx_dqy;
//     Min(4,8) = dvx_dqx;
//     Min(4,9) = -dvx_dqw;

//     Min(5,6) = -dvx_dqy;
//     Min(5,7) = -dvx_dqz;
//     Min(5,8) = dvx_dqw;
//     Min(5,9) = dvx_dqx;

//     Min(6,7) = -dgx_h;
//     Min(6,8) = -dgy_h;
//     Min(6,9) = -dgz_h;

//     Min(7,6) = dgx_h;
//     Min(7,8) = dgz_h;
//     Min(7,9) = -dgy_h;

//     Min(8,6) = dgy_h;
//     Min(8,7) = -dgz_h;
//     Min(8,9) = dgx_h;

//     Min(9,6) = dgz_h;
//     Min(9,7) = dgy_h;
//     Min(9,8) = -dgx_h;

// }

// // jacobienne de f par rapport à u
// void Fu(Vector &x, Vector &u)
// {
//     /*
//     Ju =

//     [         0,          0,          0,                              0,                              0,                              0]
//     [         0,          0,          0,                              0,                              0,                              0]
//     [         0,          0,          0,                              0,                              0,                              0]
//     [         0,          0,          0, dt*(qw^2 + qx^2 - qy^2 - qz^2),         dt*(2*qw*qz + 2*qx*qy),          -2*dt*(qw*qy - qx*qz)]
//     [         0,          0,          0,          -2*dt*(qw*qz - qx*qy), dt*(qw^2 - qx^2 + qy^2 - qz^2),         dt*(2*qw*qx + 2*qy*qz)]
//     [         0,          0,          0,         dt*(2*qw*qy + 2*qx*qz),          -2*dt*(qw*qx - qy*qz), dt*(qw^2 - qx^2 - qy^2 + qz^2)]
//     [-(dt*qx)/2, -(dt*qy)/2, -(dt*qz)/2,                              0,                              0,                              0]
//     [ (dt*qw)/2, -(dt*qz)/2,  (dt*qy)/2,                              0,                              0,                              0]
//     [ (dt*qz)/2,  (dt*qw)/2, -(dt*qx)/2,                              0,                              0,                              0]
//     [-(dt*qy)/2,  (dt*qx)/2,  (dt*qw)/2,                              0,                              0,                              0]
// */

// Min.set_zero();
// Min(3,3) = dt*(x(6)*x(6) + x(7)*x(7) - x(8)*x(8) - x(9)*x(9));
// Min(3,4) = dt*(2*x(6)*x(9) + 2*x(7)*x(8));
// Min(3,5) = -2*dt*(x(6)*x(8) - x(7)*x(9));

// Min(4,3) = -2*dt*(x(6)*x(9) - x(7)*x(8));
// Min(4,4) = dt*(x(6)*x(6) - x(7)*x(7) + x(8)*x(8) - x(9)*x(9));
// Min(4,5) = dt*(2*x(6)*x(8) + 2*x(7)*x(9));

// Min(5,3) = 2*dt*(x(6)*x(8) + x(7)*x(9));
// Min(5,4) = -2*dt*(x(6)*x(8) - x(7)*x(9));
// Min(5,5) = dt*(x(6)*x(6) - x(7)*x(7) - x(8)*x(8) + x(9)*x(9));

// Min(6,0) = -dt*x(7)/2;
// Min(6,1) = -dt*x(8)/2;
// Min(6,2) = -dt*x(9)/2;

// Min(7,0) = dt*x(6)/2;
// Min(7,1) = -dt*x(9)/2;
// Min(7,2) = dt*x(8)/2;

// Min(8,0) = dt*x(9)/2;
// Min(8,1) = dt*x(6)/2;
// Min(8,2) = -dt*x(7)/2;

// Min(9,0) = -dt*x(8)/2;
// Min(9,1) = dt*x(7)/2;
// Min(9,2) = dt*x(6)/2;
// }

// // jacobienne de h par rapport à x
// void H(Vector &x)
// {
//     Min.set_eye();
// }

#define GYRO_VAR 0.001
#define ACC_VAR 0.01
#define EXT_VAR 0.1

#define X_DIM 35 // X = [x, y, z, vx, vy, vz, qw, qx, qy, qz, bgx, bgy, bgz, bax, bay, baz, sgxx, sgxy, sgxz, sgyx, sgyy, sgyz, sgzx, sgzy, sgzz, saxx, saxy, saxz, sayx, sayy, sayz, sazx, sazy, sazz, gravity]
#define Z_DIM 3  // Z = [x,y,z]
#define U_DIM 6  // U = [gx,gy,gz,ax,ay,az]
#define gravity -9.81

void f(Vector &x, Vector &u)
{
    /*
    function next_X = transitionFunction(prev_X, U, params)
        %X_est = [x y z vx vy vz qw qx qy qz] -> position, orientation and linear velocity
        %U = [gx gy gz ax ay az] ->
        %function for state transition (pour la prediction using commands
        %or proprioceptive sensors)

        [x, y, z, vx, vy, vz, qw, qx, qy, qz, bgx, bgy, bgz, bax, bay, baz, sgxx, sgxy, sgxz, sgyx, sgyy, sgyz, sgzx, sgzy, sgzz, saxx, saxy, saxz, sayx, sayy, sayz, sazx, sazy, sazz, gravity] = deal(prev_X(1),prev_X(2),prev_X(3),prev_X(4), ...
                                prev_X(5),prev_X(6),prev_X(7),prev_X(8),prev_X(9),prev_X(10),prev_X(11),prev_X(12),prev_X(13),prev_X(14), ...
                                prev_X(15),prev_X(16),prev_X(17),prev_X(18),prev_X(19),prev_X(20),prev_X(21),prev_X(22),prev_X(23),prev_X(24),prev_X(25),prev_X(26),prev_X(27),prev_X(28),prev_X(29),prev_X(30),prev_X(31),prev_X(32),prev_X(33),prev_X(34), prev_X(35));

        [gx,gy,gz,ax,ay,az] = deal(U(1),U(2),U(3),U(4),U(5),U(6));

        dt=params(1);

        x_ = x+vx*dt;
        y_ = y+vy*dt;
        z_ = z+vz*dt;					% integration of linear velocity


        ac_xyz = [(ax+bax);(ay+bay);(az+baz)];
        noa = [saxx saxy saxz; sayx sayy sayz; sazx sazy sazz];
        ac_xyz = noa * ac_xyz;

        [ae_x,ae_y,ae_z] = rotate(ac_xyz(1),ac_xyz(2),ac_xyz(3),qw,-qx,-qy,-qz);
        vx_ = vx+ae_x*dt;
        vy_ = vy+ae_y*dt;
        vz_ = vz+(ae_z-gravity)*dt;				% integration of linear acceleration


        gc_xyz = [(gx+bgx);(gy+bgy);(gz+bgz)];
        nog = [sgxx sgxy sgxz; sgyx sgyy sgyz; sgzx sgzy sgzz];
        gc_xyz = nog * gc_xyz;

        [dqw,dqx,dqy,dqz] = quatMul(qw,qx,qy,qz,0,gc_xyz(1),gc_xyz(2),gc_xyz(3));
        q_ = [qw,qx,qy,qz] + 0.5*dt*[dqw,dqx,dqy,dqz];

        next_X = [x_ y_ z_ vx_ vy_ vz_ q_(1) q_(2) q_(3) q_(4) bgx, bgy, bgz, bax, bay, baz, sgxx, sgxy, sgxz, sgyx, sgyy, sgyz, sgzx, sgzy, sgzz, saxx, saxy, saxz, sayx, sayy, sayz, sazx, sazy, sazz, gravity];
    end

    f =

        x + dt*vx
        y + dt*vy
        z + dt*vz
        vx + dt*(qw*(qw*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) - qy*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) + qz*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz))) - qy*(qw*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - qx*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + qy*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))) + qx*(qx*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) + qy*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + qz*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz))) + qz*(qw*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + qx*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - qz*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))))
        vy + dt*(qw*(qw*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + qx*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - qz*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))) + qx*(qw*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - qx*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + qy*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))) + qy*(qx*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) + qy*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + qz*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz))) - qz*(qw*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) - qy*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) + qz*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz))))
        vz + dt*(qw*(qw*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - qx*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + qy*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))) - gravity - qx*(qw*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + qx*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - qz*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))) + qy*(qw*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) - qy*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) + qz*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz))) + qz*(qx*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) + qy*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + qz*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz))))
        qw - (dt*(qx*(sgxx*(bgx + gx) + sgxy*(bgy + gy) + sgxz*(bgz + gz)) + qy*(sgyx*(bgx + gx) + sgyy*(bgy + gy) + sgyz*(bgz + gz)) + qz*(sgzx*(bgx + gx) + sgzy*(bgy + gy) + sgzz*(bgz + gz))))/2
        qx + (dt*(qw*(sgxx*(bgx + gx) + sgxy*(bgy + gy) + sgxz*(bgz + gz)) + qy*(sgzx*(bgx + gx) + sgzy*(bgy + gy) + sgzz*(bgz + gz)) - qz*(sgyx*(bgx + gx) + sgyy*(bgy + gy) + sgyz*(bgz + gz))))/2
        qy + (dt*(qw*(sgyx*(bgx + gx) + sgyy*(bgy + gy) + sgyz*(bgz + gz)) - qx*(sgzx*(bgx + gx) + sgzy*(bgy + gy) + sgzz*(bgz + gz)) + qz*(sgxx*(bgx + gx) + sgxy*(bgy + gy) + sgxz*(bgz + gz))))/2
        qz + (dt*(qw*(sgzx*(bgx + gx) + sgzy*(bgy + gy) + sgzz*(bgz + gz)) + qx*(sgyx*(bgx + gx) + sgyy*(bgy + gy) + sgyz*(bgz + gz)) - qy*(sgxx*(bgx + gx) + sgxy*(bgy + gy) + sgxz*(bgz + gz))))/2
        bgx
        bgy
        bgz
        bax
        bay
        baz
        sgxx
        sgxy
        sgxz
        sgyx
        sgyy
        sgyz
        sgzx
        sgzy
        sgzz
        saxx
        saxy
        saxz
        sayx
        sayy
        sayz
        sazx
        sazy
        sazz
        gravity



    Do not forget to add the normalization of the quaternion at the end of the function
*/

    // compensate bias and scale
    data_type cax = u(3) + x(13), cay = u(4) + x(14), caz = u(5) + x(15);
    data_type ccax = cax * x(16) + cay * x(17) + caz * x(18);
    data_type ccay = cax * x(19) + cay * x(20) + caz * x(21);
    data_type ccaz = cax * x(22) + cay * x(23) + caz * x(24);

    data_type cgx = u(0) + x(10), cgy = u(1) + x(11), cgz = u(2) + x(12);
    data_type ccgx = cgx * x(25) + cgy * x(26) + cgz * x(27);
    data_type ccgy = cgx * x(28) + cgy * x(29) + cgz * x(30);
    data_type ccgz = cgx * x(31) + cgy * x(32) + cgz * x(33);

    // intermediate variables
    data_type dax = ccax * dt, day = ccay * dt, daz = ccaz * dt;
    data_type qw2 = x(6) * x(6), qx2 = x(7) * x(7), qy2 = x(8) * x(8), qz2 = x(9) * x(9);

    // integration of linear velocity
    Vout(0) = x(0) + x(3) * dt;
    Vout(1) = x(1) + x(4) * dt;
    Vout(2) = x(2) + x(5) * dt;

    // integration of linear acceleration
    Vout(3) = x(3) + (dax * qw2 - 2 * daz * x(6) * x(8) + 2 * day * x(6) * x(9) + dax * qx2 + 2 * day * x(7) * x(8) + 2 * daz * x(7) * x(9) - dax * qy2 - dax * qz2 + u(0)) * dt;
    Vout(4) = x(4) + (day * qw2 + 2 * daz * x(6) * x(7) - 2 * dax * x(6) * x(9) - day * qx2 + 2 * dax * x(7) * x(8) + day * qy2 + 2 * daz * x(8) * x(9) - day * qz2 + u(1)) * dt;
    Vout(5) = x(5) + (daz * qw2 - 2 * day * x(6) * x(7) + 2 * dax * x(6) * x(8) - daz * qx2 + 2 * dax * x(7) * x(9) - daz * qy2 + 2 * day * x(8) * x(9) + daz * qz2 + u(2) - x(34)) * dt;

    // integration of quaternion
    Vout(6) = x(6) - (dt * (ccgx * x(7) + ccgy * x(8) + ccgz * x(9))) / 2;
    Vout(7) = x(7) + (dt * (ccgx * x(6) - ccgy * x(9) + ccgz * x(8))) / 2;
    Vout(8) = x(8) + (dt * (ccgy * x(6) + ccgx * x(9) - ccgz * x(6))) / 2;
    Vout(9) = x(9) + (dt * (ccgz * x(6) - ccgy * x(8) + ccgx * x(6))) / 2;

    // normalize quaternion
    data_type norm_inv = 1 / sqrt(Vout(6) * Vout(6) + Vout(7) * Vout(7) + Vout(8) * Vout(8) + Vout(9) * Vout(9));
    Vout(6) *= norm_inv;
    Vout(7) *= norm_inv;
    Vout(8) *= norm_inv;
    Vout(9) *= norm_inv;

    // copy bias and scale
    for (int i = 10; i < 34; i++)
        Vout(i) = x(i);
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
    /*
    without normalization of the quaternion at the end of the function (negligible error)
   Jx =

[1, 0, 0, dt,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 1, 0,  0, dt,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 1,  0,  0, dt,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  1,  0,  0, dt*(2*qw*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) - 2*qy*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) + 2*qz*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz))),  dt*(2*qx*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) + 2*qy*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qz*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz))), -dt*(2*qw*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - 2*qx*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qy*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))),  dt*(2*qw*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qx*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - 2*qz*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))),                                     0,                                     0,                                     0, dt*(saxx*qw^2 - 2*sazx*qw*qy + 2*sayx*qw*qz + saxx*qx^2 + 2*sayx*qx*qy + 2*sazx*qx*qz - saxx*qy^2 - saxx*qz^2), dt*(saxy*qw^2 - 2*sazy*qw*qy + 2*sayy*qw*qz + saxy*qx^2 + 2*sayy*qx*qy + 2*sazy*qx*qz - saxy*qy^2 - saxy*qz^2), dt*(saxz*qw^2 - 2*sazz*qw*qy + 2*sayz*qw*qz + saxz*qx^2 + 2*sayz*qx*qy + 2*sazz*qx*qz - saxz*qy^2 - saxz*qz^2),                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0, dt*(ax + bax)*(qw^2 + qx^2 - qy^2 - qz^2), dt*(ay + bay)*(qw^2 + qx^2 - qy^2 - qz^2), dt*(az + baz)*(qw^2 + qx^2 - qy^2 - qz^2),           2*dt*(ax + bax)*(qw*qz + qx*qy),           2*dt*(ay + bay)*(qw*qz + qx*qy),           2*dt*(az + baz)*(qw*qz + qx*qy),          -2*dt*(ax + bax)*(qw*qy - qx*qz),          -2*dt*(ay + bay)*(qw*qy - qx*qz),          -2*dt*(az + baz)*(qw*qy - qx*qz),   0]
[0, 0, 0,  0,  1,  0, dt*(2*qw*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qx*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - 2*qz*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))),  dt*(2*qw*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - 2*qx*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qy*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))),  dt*(2*qx*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) + 2*qy*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qz*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz))), -dt*(2*qw*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) - 2*qy*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) + 2*qz*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz))),                                     0,                                     0,                                     0, dt*(sayx*qw^2 + 2*sazx*qw*qx - 2*saxx*qw*qz - sayx*qx^2 + 2*saxx*qx*qy + sayx*qy^2 + 2*sazx*qy*qz - sayx*qz^2), dt*(sayy*qw^2 + 2*sazy*qw*qx - 2*saxy*qw*qz - sayy*qx^2 + 2*saxy*qx*qy + sayy*qy^2 + 2*sazy*qy*qz - sayy*qz^2), dt*(sayz*qw^2 + 2*sazz*qw*qx - 2*saxz*qw*qz - sayz*qx^2 + 2*saxz*qx*qy + sayz*qy^2 + 2*sazz*qy*qz - sayz*qz^2),                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,          -2*dt*(ax + bax)*(qw*qz - qx*qy),          -2*dt*(ay + bay)*(qw*qz - qx*qy),          -2*dt*(az + baz)*(qw*qz - qx*qy), dt*(ax + bax)*(qw^2 - qx^2 + qy^2 - qz^2), dt*(ay + bay)*(qw^2 - qx^2 + qy^2 - qz^2), dt*(az + baz)*(qw^2 - qx^2 + qy^2 - qz^2),           2*dt*(ax + bax)*(qw*qx + qy*qz),           2*dt*(ay + bay)*(qw*qx + qy*qz),           2*dt*(az + baz)*(qw*qx + qy*qz),   0]
[0, 0, 0,  0,  0,  1, dt*(2*qw*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - 2*qx*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qy*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))), -dt*(2*qw*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qx*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - 2*qz*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))),  dt*(2*qw*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) - 2*qy*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) + 2*qz*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz))),  dt*(2*qx*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) + 2*qy*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qz*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz))),                                     0,                                     0,                                     0, dt*(sazx*qw^2 - 2*sayx*qw*qx + 2*saxx*qw*qy - sazx*qx^2 + 2*saxx*qx*qz - sazx*qy^2 + 2*sayx*qy*qz + sazx*qz^2), dt*(sazy*qw^2 - 2*sayy*qw*qx + 2*saxy*qw*qy - sazy*qx^2 + 2*saxy*qx*qz - sazy*qy^2 + 2*sayy*qy*qz + sazy*qz^2), dt*(sazz*qw^2 - 2*sayz*qw*qx + 2*saxz*qw*qy - sazz*qx^2 + 2*saxz*qx*qz - sazz*qy^2 + 2*sayz*qy*qz + sazz*qz^2),                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,           2*dt*(ax + bax)*(qw*qy + qx*qz),           2*dt*(ay + bay)*(qw*qy + qx*qz),           2*dt*(az + baz)*(qw*qy + qx*qz),          -2*dt*(ax + bax)*(qw*qx - qy*qz),          -2*dt*(ay + bay)*(qw*qx - qy*qz),          -2*dt*(az + baz)*(qw*qx - qy*qz), dt*(ax + bax)*(qw^2 - qx^2 - qy^2 + qz^2), dt*(ay + bay)*(qw^2 - qx^2 - qy^2 + qz^2), dt*(az + baz)*(qw^2 - qx^2 - qy^2 + qz^2), -dt]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         1,                                                                                                                              -(dt*(sgxx*(bgx + gx) + sgxy*(bgy + gy) + sgxz*(bgz + gz)))/2,                                                                                                                              -(dt*(sgyx*(bgx + gx) + sgyy*(bgy + gy) + sgyz*(bgz + gz)))/2,                                                                                                                              -(dt*(sgzx*(bgx + gx) + sgzy*(bgy + gy) + sgzz*(bgz + gz)))/2, -(dt*(qx*sgxx + qy*sgyx + qz*sgzx))/2, -(dt*(qx*sgxy + qy*sgyy + qz*sgzy))/2, -(dt*(qx*sgxz + qy*sgyz + qz*sgzz))/2,                                                                                                              0,                                                                                                              0,                                                                                                              0, -(dt*qx*(bgx + gx))/2, -(dt*qx*(bgy + gy))/2, -(dt*qx*(bgz + gz))/2, -(dt*qy*(bgx + gx))/2, -(dt*qy*(bgy + gy))/2, -(dt*qy*(bgz + gz))/2, -(dt*qz*(bgx + gx))/2, -(dt*qz*(bgy + gy))/2, -(dt*qz*(bgz + gz))/2,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                              (dt*(sgxx*(bgx + gx) + sgxy*(bgy + gy) + sgxz*(bgz + gz)))/2,                                                                                                                                                                                          1,                                                                                                                               (dt*(sgzx*(bgx + gx) + sgzy*(bgy + gy) + sgzz*(bgz + gz)))/2,                                                                                                                              -(dt*(sgyx*(bgx + gx) + sgyy*(bgy + gy) + sgyz*(bgz + gz)))/2,  (dt*(qw*sgxx + qy*sgzx - qz*sgyx))/2,  (dt*(qw*sgxy + qy*sgzy - qz*sgyy))/2,  (dt*(qw*sgxz + qy*sgzz - qz*sgyz))/2,                                                                                                              0,                                                                                                              0,                                                                                                              0,  (dt*qw*(bgx + gx))/2,  (dt*qw*(bgy + gy))/2,  (dt*qw*(bgz + gz))/2, -(dt*qz*(bgx + gx))/2, -(dt*qz*(bgy + gy))/2, -(dt*qz*(bgz + gz))/2,  (dt*qy*(bgx + gx))/2,  (dt*qy*(bgy + gy))/2,  (dt*qy*(bgz + gz))/2,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                              (dt*(sgyx*(bgx + gx) + sgyy*(bgy + gy) + sgyz*(bgz + gz)))/2,                                                                                                                              -(dt*(sgzx*(bgx + gx) + sgzy*(bgy + gy) + sgzz*(bgz + gz)))/2,                                                                                                                                                                                          1,                                                                                                                               (dt*(sgxx*(bgx + gx) + sgxy*(bgy + gy) + sgxz*(bgz + gz)))/2,  (dt*(qw*sgyx - qx*sgzx + qz*sgxx))/2,  (dt*(qw*sgyy - qx*sgzy + qz*sgxy))/2,  (dt*(qw*sgyz - qx*sgzz + qz*sgxz))/2,                                                                                                              0,                                                                                                              0,                                                                                                              0,  (dt*qz*(bgx + gx))/2,  (dt*qz*(bgy + gy))/2,  (dt*qz*(bgz + gz))/2,  (dt*qw*(bgx + gx))/2,  (dt*qw*(bgy + gy))/2,  (dt*qw*(bgz + gz))/2, -(dt*qx*(bgx + gx))/2, -(dt*qx*(bgy + gy))/2, -(dt*qx*(bgz + gz))/2,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                              (dt*(sgzx*(bgx + gx) + sgzy*(bgy + gy) + sgzz*(bgz + gz)))/2,                                                                                                                               (dt*(sgyx*(bgx + gx) + sgyy*(bgy + gy) + sgyz*(bgz + gz)))/2,                                                                                                                              -(dt*(sgxx*(bgx + gx) + sgxy*(bgy + gy) + sgxz*(bgz + gz)))/2,                                                                                                                                                                                          1,  (dt*(qw*sgzx + qx*sgyx - qy*sgxx))/2,  (dt*(qw*sgzy + qx*sgyy - qy*sgxy))/2,  (dt*(qw*sgzz + qx*sgyz - qy*sgxz))/2,                                                                                                              0,                                                                                                              0,                                                                                                              0, -(dt*qy*(bgx + gx))/2, -(dt*qy*(bgy + gy))/2, -(dt*qy*(bgz + gz))/2,  (dt*qx*(bgx + gx))/2,  (dt*qx*(bgy + gy))/2,  (dt*qx*(bgz + gz))/2,  (dt*qw*(bgx + gx))/2,  (dt*qw*(bgy + gy))/2,  (dt*qw*(bgz + gz))/2,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     1,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     1,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     1,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              1,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              1,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              1,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     1,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     1,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     1,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     1,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     1,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     1,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     1,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     1,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     1,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         1,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         1,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         1,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         1,                                         0,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         1,                                         0,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         1,                                         0,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         1,                                         0,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         1,                                         0,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         1,   0]
[0, 0, 0,  0,  0,  0,                                                                                                                                                                                         0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                                                                                                                                                                          0,                                     0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                     0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,                                         0,   1]

    */

    // compensate bias
    data_type cgx = u(0) + x(10), cgy = u(1) + x(11), cgz = u(2) + x(12);
    data_type cax = u(3) + x(13), cay = u(4) + x(14), caz = u(5) + x(15);

    // intermediaire
    data_type a11 = x(6) * cax + x(7) * cay + x(8) * caz;    //(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))
    data_type a12 = x(9) * cax + x(10) * cay + x(11) * caz;  //(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz))
    data_type a13 = x(12) * cax + x(13) * cay + x(14) * caz; //(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz))

    // dt*(2*qw*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) - 2*qy*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) + 2*qz*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz))) = dt*(2*qw*a11 - 2*qy*a12 + 2*qz*a13) = 2*dt*(qw*a11 - qy*a12 + qz*a13)
    // dt*(2*qx*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz)) + 2*qy*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qz*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz))) = dt*(2*qx*a11 + 2*qy*a13 + 2*qz*a12) = 2*dt*(qx*a11 + qy*a13 + qz*a12)
    //-dt*(2*qw*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - 2*qx*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qy*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))) = -dt*(2*qw*a12 - 2*qx*a13 + 2*qy*a11) = -2*dt*(qw*a12 - qx*a13 + qy*a11)
    // dt*(2*qw*(sayx*(ax + bax) + sayy*(ay + bay) + sayz*(az + baz)) + 2*qx*(sazx*(ax + bax) + sazy*(ay + bay) + sazz*(az + baz)) - 2*qz*(saxx*(ax + bax) + saxy*(ay + bay) + saxz*(az + baz))) = dt*(2*qw*a13 + 2*qx*a12 - 2*qz*a11) = 2*dt*(qw*a13 + qx*a12 - qz*a11)

    data_type dvx_dqw = 2 * dt * (x(6) * a11 - x(8) * a12 + x(7) * a13);  // dt*(2*qw*a11 - 2*qy*a12 + 2*qz*a13) = 2*dt*(qw*a11 - qy*a12 + qz*a13)
    data_type dvx_dqx = 2 * dt * (x(7) * a11 + x(8) * a13 + x(9) * a12);  // dt*(2*qx*a11 + 2*qy*a13 + 2*qz*a12) = 2*dt*(qx*a11 + qy*a13 + qz*a12)
    data_type dvx_dqy = -2 * dt * (x(6) * a12 - x(7) * a13 + x(8) * a11); // -dt*(2*qw*a12 - 2*qx*a13 + 2*qy*a11) = -2*dt*(qw*a12 - qx*a13 + qy*a11)
    data_type dvx_dqz = 2 * dt * (x(6) * a13 + x(7) * a12 - x(8) * a11);  // dt*(2*qw*a13 + 2*qx*a12 - 2*qz*a11) = 2*dt*(qw*a13 + qx*a12 - qz*a11)

    data_type dgx_h = 0.5 * dt * (x(16) * cgx + x(17) * cgy + x(18) * cgz); //(dt*(sgxx*(bgx + gx) + sgxy*(bgy + gy) + sgxz*(bgz + gz)))/2 = 0.5*dt*(sgxx*cgx + sgxy*cgy + sgxz*cgz)
    data_type dgy_h = 0.5 * dt * (x(19) * cgx + x(20) * cgy + x(21) * cgz); //(dt*(sgyx*(bgx + gx) + sgyy*(bgy + gy) + sgyz*(bgz + gz)))/2 = 0.5*dt*(sgyx*cgx + sgyy*cgy + sgyz*cgz)
    data_type dgz_h = 0.5 * dt * (x(22) * cgx + x(23) * cgy + x(24) * cgz); //(dt*(sgzx*(bgx + gx) + sgzy*(bgy + gy) + sgzz*(bgz + gz)))/2 = 0.5*dt*(sgzx*cgx + sgzy*cgy + sgzz*cgz)

    Min.set_eye();
    Min(0, 3) = dt;
    Min(1, 4) = dt;
    Min(2, 5) = dt;

    Min(3, 6) = dvx_dqw;
    Min(3, 7) = dvx_dqx;
    Min(3, 8) = dvx_dqy;
    Min(3, 9) = dvx_dqz;

    Min(4, 6) = dvx_dqz;
    Min(4, 7) = -dvx_dqy;
    Min(4, 8) = dvx_dqx;
    Min(4, 9) = -dvx_dqw;

    Min(5, 6) = -dvx_dqy;
    Min(5, 7) = -dvx_dqz;
    Min(5, 8) = dvx_dqw;
    Min(5, 9) = dvx_dqx;

    Min(6, 7) = -dgx_h;
    Min(6, 8) = -dgy_h;
    Min(6, 9) = -dgz_h;

    Min(7, 6) = dgx_h;
    Min(7, 8) = dgz_h;
    Min(7, 9) = -dgy_h;

    Min(8, 6) = dgy_h;
    Min(8, 7) = -dgz_h;
    Min(8, 9) = dgx_h;

    Min(9, 6) = dgz_h;
    Min(9, 7) = dgy_h;
    Min(9, 8) = -dgx_h;
    /*
    -(dt*(qx*sgxx + qy*sgyx + qz*sgzx))/2, -(dt*(qx*sgxy + qy*sgyy + qz*sgzy))/2, -(dt*(qx*sgxz + qy*sgyz + qz*sgzz))/2,
    (dt*(qw*sgxx + qy*sgzx - qz*sgyx))/2,  (dt*(qw*sgxy + qy*sgzy - qz*sgyy))/2,  (dt*(qw*sgxz + qy*sgzz - qz*sgyz))/2,
    (dt*(qw*sgyx - qx*sgzx + qz*sgxx))/2,  (dt*(qw*sgyy - qx*sgzy + qz*sgxy))/2,  (dt*(qw*sgyz - qx*sgzz + qz*sgxz))/2,
    (dt*(qw*sgzx + qx*sgyx - qy*sgxx))/2,  (dt*(qw*sgzy + qx*sgyy - qy*sgxy))/2,  (dt*(qw*sgzz + qx*sgyz - qy*sgxz))/2
    */

    Min(6, 10) = -0.5 * dt * (x(7) * x(16) + x(8) * x(19) + x(9) * x(22)); //-(dt*(qx*sgxx + qy*sgyx + qz*sgzx))/2
    Min(6, 11) = -0.5 * dt * (x(7) * x(17) + x(8) * x(20) + x(9) * x(23)); //-(dt*(qx*sgxy + qy*sgyy + qz*sgzy))/2
    Min(6, 12) = -0.5 * dt * (x(7) * x(18) + x(8) * x(21) + x(9) * x(24)); //-(dt*(qx*sgxz + qy*sgyz + qz*sgzz))/2
    Min(7, 10) = 0.5 * dt * (x(6) * x(16) + x(8) * x(22) - x(9) * x(19));  //(dt*(qw*sgxx + qy*sgzx - qz*sgyx))/2
    Min(7, 11) = 0.5 * dt * (x(6) * x(17) + x(8) * x(23) - x(9) * x(20));  //(dt*(qw*sgxy + qy*sgzy - qz*sgyy))/2
    Min(7, 12) = 0.5 * dt * (x(6) * x(18) + x(8) * x(24) - x(9) * x(21));  //(dt*(qw*sgxz + qy*sgzz - qz*sgyz))/2
    Min(8, 10) = 0.5 * dt * (x(6) * x(19) - x(7) * x(22) + x(9) * x(16));  //(dt*(qw*sgyx - qx*sgzx + qz*sgxx))/2
    Min(8, 11) = 0.5 * dt * (x(6) * x(20) - x(7) * x(23) + x(9) * x(17));  //(dt*(qw*sgyy - qx*sgzy + qz*sgxy))/2
    Min(8, 12) = 0.5 * dt * (x(6) * x(21) - x(7) * x(24) + x(9) * x(18));  //(dt*(qw*sgyz - qx*sgzz + qz*sgxz))/2
    Min(9, 10) = 0.5 * dt * (x(6) * x(22) + x(7) * x(19) - x(8) * x(16));  //(dt*(qw*sgzx + qx*sgyx - qy*sgxx))/2
    Min(9, 11) = 0.5 * dt * (x(6) * x(23) + x(7) * x(20) - x(8) * x(17));  //(dt*(qw*sgzy + qx*sgyy - qy*sgxy))/2
    Min(9, 12) = 0.5 * dt * (x(6) * x(24) + x(7) * x(21) - x(8) * x(18));  //(dt*(qw*sgzz + qx*sgyz - qy*sgxz))/2

    /*
    dt*(saxx*qw^2 - 2*sazx*qw*qy + 2*sayx*qw*qz + saxx*qx^2 + 2*sayx*qx*qy + 2*sazx*qx*qz - saxx*qy^2 - saxx*qz^2), dt*(saxy*qw^2 - 2*sazy*qw*qy + 2*sayy*qw*qz + saxy*qx^2 + 2*sayy*qx*qy + 2*sazy*qx*qz - saxy*qy^2 - saxy*qz^2), dt*(saxz*qw^2 - 2*sazz*qw*qy + 2*sayz*qw*qz + saxz*qx^2 + 2*sayz*qx*qy + 2*sazz*qx*qz - saxz*qy^2 - saxz*qz^2)
    dt*(saxy*qw^2 + 2*sazy*qw*qx - 2*sayy*qw*qy + saxy*qy^2 + 2*sayy*qx*qy - 2*sazy*qy*qz + saxy*qz^2 - saxy*qx^2), dt*(sayy*qw^2 + 2*sayz*qw*qx - 2*sayy*qw*qy + sayy*qy^2 + 2*sayz*qx*qy - 2*sayy*qy*qz + sayy*qz^2 - sayy*qx^2), dt*(sayz*qw^2 + 2*sazz*qw*qx - 2*sayz*qw*qy + sayz*qy^2 + 2*sazz*qx*qy - 2*sayz*qy*qz + sayz*qz^2 - sayz*qx^2)
    dt*(sazx*qw^2 + 2*sazy*qw*qx - 2*sayx*qw*qz + sazx*qz^2 + 2*sayx*qx*qz - 2*sazy*qy*qz + sazx*qx^2 - sazx*qy^2), dt*(sayz*qw^2 + 2*sazz*qw*qx - 2*sayz*qw*qy + sayz*qz^2 + 2*sazz*qx*qz - 2*sayz*qy*qz + sayz*qx^2 - sayz*qy^2), dt*(sazz*qw^2 + 2*sazx*qw*qx - 2*sazz*qw*qz + sazz*qz^2 + 2*sazx*qx*qz - 2*sazz*qy*qz + sazz*qx^2 - sazz*qy^2)
    */

    data_type qw2 = x(6) * x(6), qx2 = x(7) * x(7), qy2 = x(8) * x(8), qz2 = x(9) * x(9);
    Min(3, 13) = dt * (x(16) * qw2 - 2 * x(22) * x(6) * x(8) + 2 * x(19) * x(6) * x(9) + x(16) * qx2 + 2 * x(19) * x(7) * x(8) + 2 * x(22) * x(7) * x(9) - x(16) * qy2 - x(16) * qz2); // dt*(saxx*qw^2 - 2*sazx*qw*qy + 2*sayx*qw*qz + saxx*qx^2 + 2*sayx*qx*qy + 2*sazx*qx*qz - saxx*qy^2 - saxx*qz^2
    Min(3, 14) = dt * (x(17) * qw2 - 2 * x(23) * x(6) * x(8) + 2 * x(20) * x(6) * x(9) + x(17) * qx2 + 2 * x(20) * x(7) * x(8) + 2 * x(23) * x(7) * x(9) - x(17) * qy2 - x(17) * qz2); // dt*(saxy*qw^2 - 2*sazy*qw*qy + 2*sayy*qw*qz + saxy*qx^2 + 2*sayy*qx*qy + 2*sazy*qx*qz - saxy*qy^2 - saxy*qz^2
    Min(3, 15) = dt * (x(18) * qw2 - 2 * x(24) * x(6) * x(8) + 2 * x(21) * x(6) * x(9) + x(18) * qx2 + 2 * x(21) * x(7) * x(8) + 2 * x(24) * x(7) * x(9) - x(18) * qy2 - x(18) * qz2); // dt*(saxz*qw^2 - 2*sazz*qw*qy + 2*sayz*qw*qz + saxz*qx^2 + 2*sayz*qx*qy + 2*sazz*qx*qz - saxz*qy^2 - saxz*qz^2

    Min(4, 13) = dt * (x(17) * qw2 + 2 * x(23) * x(6) * x(7) - 2 * x(20) * x(6) * x(8) + x(17) * qy2 + 2 * x(20) * x(7) * x(8) - 2 * x(23) * x(7) * x(9) + x(17) * qz2 - x(17) * qx2); // dt*(saxy*qw^2 + 2*sazy*qw*qx - 2*sayy*qw*qy + saxy*qy^2 + 2*sayy*qx*qy - 2*sazy*qy*qz + saxy*qz^2 - saxy*qx^2
    Min(4, 14) = dt * (x(18) * qw2 + 2 * x(24) * x(6) * x(7) - 2 * x(21) * x(6) * x(8) + x(18) * qy2 + 2 * x(21) * x(7) * x(8) - 2 * x(24) * x(7) * x(9) + x(18) * qz2 - x(18) * qx2); // dt*(sayy*qw^2 + 2*sayz*qw*qx - 2*sayy*qw*qy + sayy*qy^2 + 2*sayz*qx*qy - 2*sayy*qy*qz + sayy*qz^2 - sayy*qx^2
    Min(4, 15) = dt * (x(19) * qw2 + 2 * x(22) * x(6) * x(7) - 2 * x(19) * x(6) * x(8) + x(19) * qy2 + 2 * x(22) * x(7) * x(8) - 2 * x(19) * x(7) * x(9) + x(19) * qz2 - x(19) * qx2); // dt*(sayz*qw^2 + 2*sazz*qw*qx - 2*sayz*qw*qy + sayz*qy^2 + 2*sazz*qx*qy - 2*sayz*qy*qz + sayz*qz^2 - sayz*qx^2

    Min(5, 13) = dt * (x(18) * qw2 + 2 * x(24) * x(6) * x(9) - 2 * x(21) * x(6) * x(7) + x(18) * qz2 + 2 * x(21) * x(7) * x(9) - 2 * x(24) * x(7) * x(8) + x(18) * qx2 - x(18) * qy2); // dt*(sazx*qw^2 + 2*sazy*qw*qx - 2*sayx*qw*qz + sazx*qz^2 + 2*sayx*qx*qz - 2*sazy*qy*qz + sazx*qx^2 - sazx*qy^2
    Min(5, 14) = dt * (x(19) * qw2 + 2 * x(22) * x(6) * x(9) - 2 * x(19) * x(6) * x(7) + x(19) * qz2 + 2 * x(22) * x(7) * x(9) - 2 * x(19) * x(7) * x(8) + x(19) * qx2 - x(19) * qy2); // dt*(sayz*qw^2 + 2*sazz*qw*qx - 2*sayz*qw*qy + sayz*qz^2 + 2*sazz*qx*qz - 2*sayz*qy*qz + sayz*qx^2 - sayz*qy^2
    Min(5, 15) = dt * (x(20) * qw2 + 2 * x(23) * x(6) * x(9) - 2 * x(20) * x(6) * x(7) + x(20) * qz2 + 2 * x(23) * x(7) * x(9) - 2 * x(20) * x(7) * x(8) + x(20) * qx2 - x(20) * qy2); // dt*(sazz*qw^2 + 2*sazx*qw*qx - 2*sazz*qw*qz + sazz*qz^2 + 2*sazx*qx*qz - 2*sazz*qy*qz + sazz*qx^2 - sazz*qy^2

    /*
    -(dt*qx*(bgx + gx))/2, -(dt*qx*(bgy + gy))/2, -(dt*qx*(bgz + gz))/2, -(dt*qy*(bgx + gx))/2, -(dt*qy*(bgy + gy))/2, -(dt*qy*(bgz + gz))/2, -(dt*qz*(bgx + gx))/2, -(dt*qz*(bgy + gy))/2, -(dt*qz*(bgz + gz))/2
    (dt*qw*(bgx + gx))/2,  (dt*qw*(bgy + gy))/2,  (dt*qw*(bgz + gz))/2, -(dt*qz*(bgx + gx))/2, -(dt*qz*(bgy + gy))/2, -(dt*qz*(bgz + gz))/2,  (dt*qy*(bgx + gx))/2,  (dt*qy*(bgy + gy))/2,  (dt*qy*(bgz + gz))/2
    (dt*qz*(bgx + gx))/2,  (dt*qz*(bgy + gy))/2,  (dt*qz*(bgz + gz))/2,  (dt*qy*(bgx + gx))/2,  (dt*qy*(bgy + gy))/2,  (dt*qy*(bgz + gz))/2, -(dt*qx*(bgx + gx))/2, -(dt*qx*(bgy + gy))/2, -(dt*qx*(bgz + gz))/2
    -(dt*qy*(bgx + gx))/2, -(dt*qy*(bgy + gy))/2, -(dt*qy*(bgz + gz))/2,  (dt*qx*(bgx + gx))/2,  (dt*qx*(bgy + gy))/2,  (dt*qx*(bgz + gz))/2,  (dt*qw*(bgx + gx))/2,  (dt*qw*(bgy + gy))/2,  (dt*qw*(bgz + gz))/2
    */

    Min(6, 16) = -0.5 * dt * x(7) * cgx; //-(dt*qx*(bgx + gx))/2
    Min(6, 17) = -0.5 * dt * x(8) * cgx; //-(dt*qx*(bgy + gy))/2
    Min(6, 18) = -0.5 * dt * x(9) * cgx; //-(dt*qx*(bgz + gz))/2
    Min(6, 19) = -0.5 * dt * x(7) * cgy; //-(dt*qy*(bgx + gx))/2
    Min(6, 20) = -0.5 * dt * x(8) * cgy; //-(dt*qy*(bgy + gy))/2
    Min(6, 21) = -0.5 * dt * x(9) * cgy; //-(dt*qy*(bgz + gz))/2
    Min(6, 22) = -0.5 * dt * x(7) * cgz; //-(dt*qz*(bgx + gx))/2
    Min(6, 23) = -0.5 * dt * x(8) * cgz; //-(dt*qz*(bgy + gy))/2
    Min(6, 24) = -0.5 * dt * x(9) * cgz; //-(dt*qz*(bgz + gz))/2
    Min(7, 16) = -Min(6, 22);            //(dt*qw*(bgx + gx))/2
    Min(7, 17) = -Min(6, 23);            //(dt*qw*(bgy + gy))/2
    Min(7, 18) = -Min(6, 24);            //(dt*qw*(bgz + gz))/2
    Min(7, 19) = -Min(6, 22);            //-(dt*qz*(bgx + gx))/2
    Min(7, 20) = -Min(6, 23);            //-(dt*qz*(bgy + gy))/2
    Min(7, 21) = -Min(6, 24);            //-(dt*qz*(bgz + gz))/2
    Min(7, 22) = Min(6, 19);             //(dt*qy*(bgx + gx))/2
    Min(7, 23) = Min(6, 20);             //(dt*qy*(bgy + gy))/2
    Min(7, 24) = Min(6, 21);             //(dt*qy*(bgz + gz))/2
    Min(8, 16) = Min(6, 23);             //(dt*qz*(bgx + gx))/2
    Min(8, 17) = Min(6, 24);             //(dt*qz*(bgy + gy))/2
    Min(8, 18) = Min(6, 22);             //(dt*qz*(bgz + gz))/2
    Min(8, 19) = -Min(6, 21);            //-(dt*qy*(bgx + gx))/2
    Min(8, 20) = -Min(6, 19);            //-(dt*qy*(bgy + gy))/2
    Min(8, 21) = -Min(6, 20);            //-(dt*qy*(bgz + gz))/2
    Min(8, 22) = -Min(6, 18);            //-(dt*qx*(bgx + gx))/2
    Min(8, 23) = -Min(6, 16);            //-(dt*qx*(bgy + gy))/2
    Min(8, 24) = -Min(6, 17);            //-(dt*qx*(bgz + gz))/2
    Min(9, 16) = -Min(6, 20);            //-(dt*qy*(bgx + gx))/2
    Min(9, 17) = -Min(6, 21);            //-(dt*qy*(bgy + gy))/2
    Min(9, 18) = -Min(6, 19);            //-(dt*qy*(bgz + gz))/2
    Min(9, 19) = Min(6, 18);             //(dt*qx*(bgx + gx))/2
    Min(9, 20) = Min(6, 16);             //(dt*qx*(bgy + gy))/2
    Min(9, 21) = Min(6, 17);             //(dt*qx*(bgz + gz))/2
    Min(9, 22) = Min(6, 24);             //(dt*qw*(bgx + gx))/2
    Min(9, 23) = Min(6, 22);             //(dt*qw*(bgy + gy))/2
    Min(9, 24) = Min(6, 23);             //(dt*qw*(bgz + gz))/2

    /*
    dt*(ax + bax)*(qw^2 + qx^2 - qy^2 - qz^2), dt*(ay + bay)*(qw^2 + qx^2 - qy^2 - qz^2), dt*(az + baz)*(qw^2 + qx^2 - qy^2 - qz^2),           2*dt*(ax + bax)*(qw*qz + qx*qy),           2*dt*(ay + bay)*(qw*qz + qx*qy),           2*dt*(az + baz)*(qw*qz + qx*qy),          -2*dt*(ax + bax)*(qw*qy - qx*qz),          -2*dt*(ay + bay)*(qw*qy - qx*qz),          -2*dt*(az + baz)*(qw*qy - qx*qz)
    2*dt*(ax + bax)*(qw*qy + qx*qz),           2*dt*(ay + bay)*(qw*qy + qx*qz),           2*dt*(az + baz)*(qw*qy + qx*qz),          -2*dt*(ax + bax)*(qw*qx - qy*qz),          -2*dt*(ay + bay)*(qw*qx - qy*qz),          -2*dt*(az + baz)*(qw*qx - qy*qz), dt*(ax + bax)*(qw^2 - qx^2 - qy^2 + qz^2), dt*(ay + bay)*(qw^2 - qx^2 - qy^2 + qz^2), dt*(az + baz)*(qw^2 - qx^2 - qy^2 + qz^2)
    -2*dt*(ax + bax)*(qw*qz - qx*qy),          -2*dt*(ay + bay)*(qw*qz - qx*qy),          -2*dt*(az + baz)*(qw*qz - qx*qy), dt*(ax + bax)*(qw^2 - qx^2 + qy^2 - qz^2), dt*(ay + bay)*(qw^2 - qx^2 + qy^2 - qz^2), dt*(az + baz)*(qw^2 - qx^2 + qy^2 - qz^2),           2*dt*(ax + bax)*(qw*qx + qy*qz),           2*dt*(ay + bay)*(qw*qx + qy*qz),           2*dt*(az + baz)*(qw*qx + qy*qz)
    */

    Min(3, 25) = dt * cax * (qw2 + qx2 - qy2 - qz2);          // dt*(ax + bax)*(qw^2 + qx^2 - qy^2 - qz^2)
    Min(3, 26) = dt * cay * (qw2 + qx2 - qy2 - qz2);          // dt*(ay + bay)*(qw^2 + qx^2 - qy^2 - qz^2)
    Min(3, 27) = dt * caz * (qw2 + qx2 - qy2 - qz2);          // dt*(az + baz)*(qw^2 + qx^2 - qy^2 - qz^2)
    Min(3, 28) = 2 * dt * cax * (x(6) * x(9) + x(7) * x(8));  // 2*dt*(ax + bax)*(qw*qz + qx*qy)
    Min(3, 29) = 2 * dt * cay * (x(6) * x(9) + x(7) * x(8));  // 2*dt*(ay + bay)*(qw*qz + qx*qy)
    Min(3, 30) = 2 * dt * caz * (x(6) * x(9) + x(7) * x(8));  // 2*dt*(az + baz)*(qw*qz + qx*qy)
    Min(3, 31) = -2 * dt * cax * (x(7) * x(9) - x(6) * x(8)); //-2*dt*(ax + bax)*(qw*qy - qx*qz)
    Min(3, 32) = -2 * dt * cay * (x(7) * x(9) - x(6) * x(8)); //-2*dt*(ay + bay)*(qw*qy - qx*qz)
    Min(3, 33) = -2 * dt * caz * (x(7) * x(9) - x(6) * x(8)); //-2*dt*(az + baz)*(qw*qy - qx*qz)

    Min(4, 25) = 2 * dt * cax * (x(7) * x(9) + x(6) * x(8));  // 2*dt*(ax + bax)*(qw*qy + qx*qz)
    Min(4, 26) = 2 * dt * cay * (x(7) * x(9) + x(6) * x(8));  // 2*dt*(ay + bay)*(qw*qy + qx*qz)
    Min(4, 27) = 2 * dt * caz * (x(7) * x(9) + x(6) * x(8));  // 2*dt*(az + baz)*(qw*qy + qx*qz)
    Min(4, 28) = -2 * dt * cax * (x(8) * x(9) - x(6) * x(7)); //-2*dt*(ax + bax)*(qw*qx - qy*qz)
    Min(4, 29) = -2 * dt * cay * (x(8) * x(9) - x(6) * x(7)); //-2*dt*(ay + bay)*(qw*qx - qy*qz)
    Min(4, 30) = -2 * dt * caz * (x(8) * x(9) - x(6) * x(7)); //-2*dt*(az + baz)*(qw*qx - qy*qz)
    Min(4, 31) = dt * cax * (qw2 - qx2 - qy2 + qz2);          // dt*(ax + bax)*(qw^2 - qx^2 - qy^2 + qz^2)
    Min(4, 32) = dt * cay * (qw2 - qx2 - qy2 + qz2);          // dt*(ay + bay)*(qw^2 - qx^2 - qy^2 + qz^2)
    Min(4, 33) = dt * caz * (qw2 - qx2 - qy2 + qz2);          // dt*(az + baz)*(qw^2 - qx^2 - qy^2 + qz^2)

    Min(5, 25) = -2 * dt * cax * (x(8) * x(9) + x(6) * x(7)); //-2*dt*(ax + bax)*(qw*qz - qx*qy)
    Min(5, 26) = -2 * dt * cay * (x(8) * x(9) + x(6) * x(7)); //-2*dt*(ay + bay)*(qw*qz - qx*qy)
    Min(5, 27) = -2 * dt * caz * (x(8) * x(9) + x(6) * x(7)); //-2*dt*(az + baz)*(qw*qz - qx*qy)
    Min(5, 28) = dt * cax * (qw2 - qx2 + qy2 - qz2);          // dt*(ax + bax)*(qw^2 - qx^2 + qy^2 - qz^2)
    Min(5, 29) = dt * cay * (qw2 - qx2 + qy2 - qz2);          // dt*(ay + bay)*(qw^2 - qx^2 + qy^2 - qz^2)
    Min(5, 30) = dt * caz * (qw2 - qx2 + qy2 - qz2);          // dt*(az + baz)*(qw^2 - qx^2 + qy^2 - qz^2)
    Min(5, 31) = 2 * dt * cax * (x(6) * x(9) + x(7) * x(8));  // 2*dt*(ax + bax)*(qw*qx + qy*qz)
    Min(5, 32) = 2 * dt * cay * (x(6) * x(9) + x(7) * x(8));  // 2*dt*(ay + bay)*(qw*qx + qy*qz)
    Min(5, 33) = 2 * dt * caz * (x(6) * x(9) + x(7) * x(8));  // 2*dt*(az + baz)*(qw*qx + qy*qz)

    Min(5, 34) = dt;
}   

// jacobienne de f par rapport à u
void Fu(Vector &x, Vector &u)
{
    /*
   Ju =
 
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0, dt*(saxx*qw^2 - 2*sazx*qw*qy + 2*sayx*qw*qz + saxx*qx^2 + 2*sayx*qx*qy + 2*sazx*qx*qz - saxx*qy^2 - saxx*qz^2), dt*(saxy*qw^2 - 2*sazy*qw*qy + 2*sayy*qw*qz + saxy*qx^2 + 2*sayy*qx*qy + 2*sazy*qx*qz - saxy*qy^2 - saxy*qz^2), dt*(saxz*qw^2 - 2*sazz*qw*qy + 2*sayz*qw*qz + saxz*qx^2 + 2*sayz*qx*qy + 2*sazz*qx*qz - saxz*qy^2 - saxz*qz^2)]
[                                    0,                                     0,                                     0, dt*(sayx*qw^2 + 2*sazx*qw*qx - 2*saxx*qw*qz - sayx*qx^2 + 2*saxx*qx*qy + sayx*qy^2 + 2*sazx*qy*qz - sayx*qz^2), dt*(sayy*qw^2 + 2*sazy*qw*qx - 2*saxy*qw*qz - sayy*qx^2 + 2*saxy*qx*qy + sayy*qy^2 + 2*sazy*qy*qz - sayy*qz^2), dt*(sayz*qw^2 + 2*sazz*qw*qx - 2*saxz*qw*qz - sayz*qx^2 + 2*saxz*qx*qy + sayz*qy^2 + 2*sazz*qy*qz - sayz*qz^2)]
[                                    0,                                     0,                                     0, dt*(sazx*qw^2 - 2*sayx*qw*qx + 2*saxx*qw*qy - sazx*qx^2 + 2*saxx*qx*qz - sazx*qy^2 + 2*sayx*qy*qz + sazx*qz^2), dt*(sazy*qw^2 - 2*sayy*qw*qx + 2*saxy*qw*qy - sazy*qx^2 + 2*saxy*qx*qz - sazy*qy^2 + 2*sayy*qy*qz + sazy*qz^2), dt*(sazz*qw^2 - 2*sayz*qw*qx + 2*saxz*qw*qy - sazz*qx^2 + 2*saxz*qx*qz - sazz*qy^2 + 2*sayz*qy*qz + sazz*qz^2)]
[-(dt*(qx*sgxx + qy*sgyx + qz*sgzx))/2, -(dt*(qx*sgxy + qy*sgyy + qz*sgzy))/2, -(dt*(qx*sgxz + qy*sgyz + qz*sgzz))/2,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[ (dt*(qw*sgxx + qy*sgzx - qz*sgyx))/2,  (dt*(qw*sgxy + qy*sgzy - qz*sgyy))/2,  (dt*(qw*sgxz + qy*sgzz - qz*sgyz))/2,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[ (dt*(qw*sgyx - qx*sgzx + qz*sgxx))/2,  (dt*(qw*sgyy - qx*sgzy + qz*sgxy))/2,  (dt*(qw*sgyz - qx*sgzz + qz*sgxz))/2,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[ (dt*(qw*sgzx + qx*sgyx - qy*sgxx))/2,  (dt*(qw*sgzy + qx*sgyy - qy*sgxy))/2,  (dt*(qw*sgzz + qx*sgyz - qy*sgxz))/2,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
[                                    0,                                     0,                                     0,                                                                                                              0,                                                                                                              0,                                                                                                              0]
 
*/
    
    data_type qw2 = x(6) * x(6), qx2 = x(7) * x(7), qy2 = x(8) * x(8), qz2 = x(9) * x(9);
    Min.set_zero();
    Min(3, 3) = dt * (x(16) * qw2 - 2 * x(22) * x(6) * x(8) + 2 * x(19) * x(6) * x(9) + x(16) * qx2 + 2 * x(19) * x(7) * x(8) + 2 * x(22) * x(7) * x(9) - x(16) * qy2 - x(16) * qz2); // dt*(saxx*qw^2 - 2*sazx*qw*qy + 2*sayx*qw*qz + saxx*qx^2 + 2*sayx*qx*qy + 2*sazx*qx*qz - saxx*qy^2 - saxx*qz^2)
    Min(3, 4) = dt * (x(17) * qw2 - 2 * x(23) * x(6) * x(8) + 2 * x(20) * x(6) * x(9) + x(17) * qx2 + 2 * x(20) * x(7) * x(8) + 2 * x(23) * x(7) * x(9) - x(17) * qy2 - x(17) * qz2); // dt*(saxy*qw^2 - 2*sazy*qw*qy + 2*sayy*qw*qz + saxy*qx^2 + 2*sayy*qx*qy + 2*sazy*qx*qz - saxy*qy^2 - saxy*qz^2)
    Min(3, 5) = dt * (x(18) * qw2 - 2 * x(24) * x(6) * x(8) + 2 * x(21) * x(6) * x(9) + x(18) * qx2 + 2 * x(21) * x(7) * x(8) + 2 * x(24) * x(7) * x(9) - x(18) * qy2 - x(18) * qz2); // dt*(saxz*qw^2 - 2*sazz*qw*qy + 2*sayz*qw*qz + saxz*qx^2 + 2*sayz*qx*qy + 2*sazz*qx*qz - saxz*qy^2 - saxz*qz^2)

    Min(4, 3) = dt * (x(17) * qw2 + 2 * x(23) * x(6) * x(7) - 2 * x(20) * x(6) * x(8) + x(17) * qy2 + 2 * x(20) * x(7) * x(8) - 2 * x(23) * x(7) * x(9) + x(17) * qz2 - x(17) * qx2); // dt*(saxy*qw^2 + 2*sazy*qw*qx - 2*sayy*qw*qy + saxy*qy^2 + 2*sayy*qx*qy - 2*sazy*qy*qz + saxy*qz^2 - saxy*qx^2
    Min(4, 4) = dt * (x(18) * qw2 + 2 * x(24) * x(6) * x(7) - 2 * x(21) * x(6) * x(8) + x(18) * qy2 + 2 * x(21) * x(7) * x(8) - 2 * x(24) * x(7) * x(9) + x(18) * qz2 - x(18) * qx2); // dt*(sayy*qw^2 + 2*sayz*qw*qx - 2*sayy*qw*qy + sayy*qy^2 + 2*sayz*qx*qy - 2*sayy*qy*qz + sayy*qz^2 - sayy*qx^2
    Min(4, 5) = dt * (x(19) * qw2 + 2 * x(22) * x(6) * x(7) - 2 * x(19) * x(6) * x(8) + x(19) * qy2 + 2 * x(22) * x(7) * x(8) - 2 * x(19) * x(7) * x(9) + x(19) * qz2 - x(19) * qx2); // dt*(sayz*qw^2 + 2*sazz*qw*qx - 2*sayz*qw*qy + sayz*qy^2 + 2*sazz*qx*qy - 2*sayz*qy*qz + sayz*qz^2 - sayz*qx^2

    Min(5, 3) = dt * (x(18) * qw2 + 2 * x(24) * x(6) * x(9) - 2 * x(21) * x(6) * x(7) + x(18) * qz2 + 2 * x(21) * x(7) * x(9) - 2 * x(24) * x(7) * x(8) + x(18) * qx2 - x(18) * qy2); // dt*(sazx*qw^2 + 2*sazy*qw*qx - 2*sayx*qw*qz + sazx*qz^2 + 2*sayx*qx*qz - 2*sazy*qy*qz + sazx*qx^2 - sazx*qy^2
    Min(5, 4) = dt * (x(19) * qw2 + 2 * x(22) * x(6) * x(9) - 2 * x(19) * x(6) * x(7) + x(19) * qz2 + 2 * x(22) * x(7) * x(9) - 2 * x(19) * x(7) * x(8) + x(19) * qx2 - x(19) * qy2); // dt*(sayz*qw^2 + 2*sazz*qw*qx - 2*sayz*qw*qy + sayz*qz^2 + 2*sazz*qx*qz - 2*sayz*qy*qz + sayz*qx^2 - sayz*qy^2
    Min(5, 5) = dt * (x(20) * qw2 + 2 * x(23) * x(6) * x(9) - 2 * x(20) * x(6) * x(7) + x(20) * qz2 + 2 * x(23) * x(7) * x(9) - 2 * x(20) * x(7) * x(8) + x(20) * qx2 - x(20) * qy2); // dt*(sazz*qw^2 + 2*sazx*qw*qx - 2*sazz*qw*qz + sazz*qz^2 + 2*sazx*qx*qz - 2*sazz*qy*qz + sazz*qx^2 - sazz*qy^2

    Min(6, 1) = -0.5 * dt * (x(7) * x(16) + x(8) * x(17) + x(9) * x(18)); //-(dt*(qx*sgxx + qy*sgyx + qz*sgzx))/2
    Min(6, 2) = -0.5 * dt * (x(7) * x(19) + x(8) * x(20) + x(9) * x(21)); //-(dt*(qx*sgxy + qy*sgyy + qz*sgzy))/2
    Min(6, 3) = -0.5 * dt * (x(7) * x(22) + x(8) * x(23) + x(9) * x(24)); //-(dt*(qx*sgxz + qy*sgyz + qz*sgzz))/2
    
    Min(7, 1) = 0.5 * dt * (x(6) * x(16) + x(8) * x(18) - x(7) * x(17)); // (dt*(qw*sgxx + qy*sgzx - qz*sgyx))/2
    Min(7, 2) = 0.5 * dt * (x(6) * x(19) + x(8) * x(21) - x(7) * x(20)); // (dt*(qw*sgxy + qy*sgzy - qz*sgyy))/2
    Min(7, 3) = 0.5 * dt * (x(6) * x(22) + x(8) * x(24) - x(7) * x(23)); // (dt*(qw*sgxz + qy*sgzz - qz*sgyz))/2

    Min(8, 1) = 0.5 * dt * (x(6) * x(17) - x(7) * x(16) + x(9) * x(18)); // (dt*(qw*sgyx - qx*sgzx + qz*sgxx))/2
    Min(8, 2) = 0.5 * dt * (x(6) * x(20) - x(7) * x(19) + x(9) * x(21)); // (dt*(qw*sgyy - qx*sgzy + qz*sgxy))/2
    Min(8, 3) = 0.5 * dt * (x(6) * x(23) - x(7) * x(22) + x(9) * x(24)); // (dt*(qw*sgyz - qx*sgzz + qz*sgxz))/2

    Min(9, 1) = 0.5 * dt * (x(6) * x(18) + x(7) * x(16) - x(8) * x(17)); // (dt*(qw*sgzx + qx*sgyx - qy*sgxx))/2
    Min(9, 2) = 0.5 * dt * (x(6) * x(21) + x(7) * x(19) - x(8) * x(20)); // (dt*(qw*sgzy + qx*sgyy - qy*sgxy))/2
    Min(9, 3) = 0.5 * dt * (x(6) * x(24) + x(7) * x(22) - x(8) * x(23)); // (dt*(qw*sgzz + qx*sgyz - qy*sgxz))/2


}

// jacobienne de h par rapport à x
void H(Vector &x)
{
    Min.set_eye();
}

#define INIT_POS_VAR 1e-6
#define INIT_VEL_VAR 1e-6
#define INIT_ATT_VAR 1e-6
#define INIT_BIAS_VAR 0//0.2*0.2
#define INIT_SCALE_VAR 0//0.02*0.02
#define INIT_GRAVITY_VAR 0.1*0.1


void initXPRQ(Vector &x, Matrix &P, Matrix &R, Matrix &Q)
{
    x(0) = 0;// x
    x(1) = 0;// y
    x(2) = 0;// z
    x(3) = 0;// vx
    x(4) = 0;// vy
    x(5) = 0;// vz
    x(6) = 1;// qw
    x(7) = 0;// qx
    x(8) = 0;// qy
    x(9) = 0;// qz
    // do not chan bias
    // x(10) = 0;// bgx
    // x(11) = 0;// bgy
    // x(12) = 0;// bgz
    // x(13) = 0;// bax
    // x(14) = 0;// bay
    // x(15) = 0;// baz
    x(16) = 1;// sgxx
    x(17) = 0;// sgxy
    x(18) = 0;// sgxz
    x(19) = 0;// sgyx
    x(20) = 1;// sgyy
    x(21) = 0;// sgyz
    x(22) = 0;// sgzx
    x(23) = 0;// sgzy
    x(24) = 1;// sgzz
    x(25) = 1;// saxx
    x(26) = 0;// saxy
    x(27) = 0;// saxz
    x(28) = 0;// sayx
    x(29) = 1;// sayy
    x(30) = 0;// sayz
    x(31) = 0;// sazx
    x(32) = 0;// sazy
    x(33) = 1;// sazz
    x(34) = gravity;// g

    P.set_zero();
    P(0, 0) =  INIT_POS_VAR;
    P(1, 1) =  INIT_POS_VAR;
    P(2, 2) =  INIT_POS_VAR;
    P(3, 3) =  INIT_VEL_VAR;
    P(4, 4) =  INIT_VEL_VAR;
    P(5, 5) =  INIT_VEL_VAR;
    P(6, 6) =  INIT_ATT_VAR;
    P(7, 7) =  INIT_ATT_VAR;
    P(8, 8) =  INIT_ATT_VAR;
    P(9, 9) =  INIT_ATT_VAR;
    P(10, 10) =  INIT_BIAS_VAR;
    P(11, 11) =  INIT_BIAS_VAR;
    P(12, 12) =  INIT_BIAS_VAR;
    P(13, 13) =  INIT_BIAS_VAR;
    P(14, 14) =  INIT_BIAS_VAR;
    P(15, 15) =  INIT_BIAS_VAR;
    P(16, 16) =  INIT_SCALE_VAR;
    P(17, 17) =  INIT_SCALE_VAR;
    P(18, 18) =  INIT_SCALE_VAR;
    P(19, 19) =  INIT_SCALE_VAR;
    P(20, 20) =  INIT_SCALE_VAR;
    P(21, 21) =  INIT_SCALE_VAR;
    P(22, 22) =  INIT_SCALE_VAR;
    P(23, 23) =  INIT_SCALE_VAR;
    P(24, 24) =  INIT_SCALE_VAR;
    P(25, 25) =  INIT_SCALE_VAR;
    P(26, 26) =  INIT_SCALE_VAR;
    P(27, 27) =  INIT_SCALE_VAR;
    P(28, 28) =  INIT_SCALE_VAR;
    P(29, 29) =  INIT_SCALE_VAR;
    P(30, 30) =  INIT_SCALE_VAR;
    P(31, 31) =  INIT_SCALE_VAR;
    P(32, 32) =  INIT_SCALE_VAR;
    P(33, 33) =  INIT_SCALE_VAR;
    P(34, 34) =  INIT_GRAVITY_VAR;

    R.set_zero();
    R(0, 0) = EXT_VAR;
    R(1, 1) = EXT_VAR;
    R(2, 2) = EXT_VAR;

    Q.set_zero();
    Q(0, 0) = GYRO_VAR;
    Q(1, 1) = GYRO_VAR;
    Q(2, 2) = GYRO_VAR;
    Q(3, 3) = ACC_VAR;
    Q(4, 4) = ACC_VAR;
    Q(5, 5) = ACC_VAR;

}
#endif // STATEESTIMATECONFIG_HPP