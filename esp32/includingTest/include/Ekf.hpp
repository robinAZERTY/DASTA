#ifndef EKF_HPP
#define EKF_HPP

/*
The Ekf class is an implementation of the Extended Kalman Filter (EKF), a recursive filter that estimates the state of a dynamic system from a series of noisy measurements.
Multiple (and unsynchronized) measurements and automatic computation of Jacobians are supported.

In summary, you can use the Ekf class to estimate the state of a system given
 - state transition (model of the system's dynamics)
 - measurement functions (measurements)
 - Jacobians (optional)

functional methods:
 - predict() -> propagate the state thanks to the command u (or proprioceptive measurements)
 - predictMeasurement(iz) -> compute the measurement h_val[iz] and its covariance corresponding to the current state x
 - mahalanobis(z, zR, iz) -> compute the probabilist square distance between a measurement z (with covariance zR) and the current estimated state
 - update(iz) -> update the state thanks to the measurement z[iz] (or exteroceptive measurements)

 If needed, you can call predictMeasurement(iz), or mahalanobis() before update(iz,false), to resolve the measurement identification problem (when you don't receive the measurement in the same order as the measurement functions).


This class uses not only Matrix and Vector classes, but also DiagoMatrix, SymMatrix and LdlMatrix classes. This provides a better RAM management and faster computation, but takes little more Flash memory.

the LdlMatrix class is used to compute the inverse of symmetric matrices in a more efficient way than the Matrix class.
The mahalanobis() method can be used to estimate the probability of the measurement.

Next step, implement the Unscented Kalman Filter (UKF) and compare the results and the computation time. -> need to implement the standard Cholesky decomposition
*/

#include "LdlMatrix.hpp"

typedef void (*Matrix_f2)(const Vector &, const Vector &, const Vector &);
typedef void (*Matrix_f1)(const Vector &, const Vector &);

class Ekf
{
protected:
    static unsigned int instance_count;
    static Matrix K; // kalman gain
    static Vector y; // inovation
    static Matrix Fx_val, Fu_val;
    static Vector utmp;      // temporary Vector used to compute Fu_val
    static LdlMatrix tmpLdl; // temporary SymMatrix
    static SymMatrix tmpSym;

    uint_fast8_t x_dim;            // state dimension
    uint_fast8_t u_dim;            // command dimension
    Matrix_f2 f = nullptr;         // state transition function
    uint_fast8_t z_number;         // number of measurements functions
    uint_fast8_t *z_dim = nullptr; // dimension of each measurement function
    Matrix_f1 *h = nullptr;        // measurement functions
    Matrix_f2 Fx = nullptr;        // jacobian function of f
    Matrix_f2 Fu = nullptr;        // jacobian function of f
    Matrix_f1 *H = nullptr;        // jacobian functions of h

    DiagoMatrix I; // identity matrix
    Matrix *H_val; // Matrix to store the jacobians
    LdlMatrix *S;  // inovation covariance matrix for each measurement prediction

    bool *h_predicted; // true if h_val was computed on the current state
    bool *S_computed;  // true if S was computed on the current state

    void allocTmp(const uint_fast8_t dim);

    void finite_diff_Fx(const uint_fast8_t i, const data_type eps = 1e-4);
    void finite_diff_Fu(const uint_fast8_t i, const data_type eps = 1e-4);
    void finite_diff_H(const uint_fast8_t iz, const uint_fast8_t i, const data_type eps = 1e-4);
    void compute_Fx_Fu();
    void finite_diff_Fx();
    void finite_diff_Fu();
    void finite_diff_H(const uint_fast8_t iz = 0);
    void compute_S(const uint_fast8_t iz = 0);

public:
    static uint_fast8_t max_x_dim, max_u_dim, max_z_dim, max_dim; // max dimensions common to all instances
    uint8_t max_z_dim_here = 0;                                   // max dimension of the measurement functions for this instance

    static Matrix tmp1, tmp2; // temporary Matrix
    static Matrix feedM;      // temporary Matrix (feeded by functions)
    static Vector feedV;      // temporary Vector (feeded by functions)

    ///////////////////////////
    ////// initialization /////
    ///////////////////////////

    // constructor and configurators

    Ekf();
    void alloc(uint_fast8_t x_dim, uint_fast8_t u_dim, uint_fast8_t z_num);
    void alloc(uint_fast8_t z_dim, uint_fast8_t iz = 0);
    void alloc(uint_fast8_t x_dim, uint_fast8_t u_dim, uint_fast8_t z_num, uint_fast8_t z_dim[]);
    Ekf(uint_fast8_t x_dim, uint_fast8_t u_dim, uint_fast8_t z_num = 1);
    void setTransitionFunction(Matrix_f2 f);
    Ekf(uint_fast8_t x_dim, uint_fast8_t u_dim, Matrix_f2 f, uint_fast8_t z_num = 1);
    void setMeasurementFunction(Matrix_f1 h, uint_fast8_t z_dim, uint_fast8_t iz = 0);
    void setMeasurementFunctions(Matrix_f1 h[], uint_fast8_t z_dim[]);
    Ekf(uint_fast8_t x_dim, uint_fast8_t u_dim, Matrix_f2 f, uint_fast8_t z_dim, Matrix_f1 h);
    void setJacobianFunction_Fx(Matrix_f2 Fx);                    // optional
    void setJacobianFunction_Fu(Matrix_f2 Fu);                    // optional
    void setJacobianFunction_H(Matrix_f1 H, uint_fast8_t iz = 0); // optional
    void setJacobianFunctions_H(Matrix_f1 H[]);                   // optional
    Ekf(uint_fast8_t x_dim, uint_fast8_t u_dim, Matrix_f2 f, uint_fast8_t z_num, uint_fast8_t z_dim[], Matrix_f1 h[], Matrix_f2 Fx = nullptr, Matrix_f2 Fu = nullptr, Matrix_f1 H[] = nullptr);
    ~Ekf();

    // firstly, set the initial state and covariance
    Vector x;    // state
    SymMatrix P; // state covariance

    // then, set the command noise covariance matrices and the measurement noise covariance matrices
    DiagoMatrix Q; // command noise (uncorrelated)
    SymMatrix *R;  // measurement covariance (can be correlated)

    // do not forget to set the constant vectors pointer if used in the functions
    Vector c; // constant (to feed functions)

    ///////////////////////////
    ///// prediction step /////
    ///////////////////////////

    // before each prediction, set the command u (or the proprioceptive measurements)
    Vector u; // command

    // propagate the state thanks to the command u (or proprioceptive measurements)
    void predict();

    /////////////////////////
    ///// update step ///////
    /////////////////////////

    // before each update, set the measurement z[iz] (or exteroceptive measurements)
    Vector *z;     // measurements
    Vector *h_val; // measurement prediction

    // compute the measurement h_val[iz] and its covariance corresponding to the current state x
    void predictMeasurement(const uint_fast8_t iz = 0);

    // compute the ~likelihood of a measurement z given the current measurement prediction h_val[iz] and its covariance S[iz]
    data_type mahalanobis(const Vector &z, const SymMatrix &zR, const uint_fast8_t iz);

    // update the state thanks to the measurement z[iz] (or exteroceptive measurements)
    void update(const uint_fast8_t iz = 0);


    //other accessors
    const uint_fast8_t &get_x_dim() const { return x_dim; }
    const uint_fast8_t &get_u_dim() const { return u_dim; }
    const uint_fast8_t &get_z_number() const { return z_number; }
    const uint_fast8_t &get_z_dim(const uint_fast8_t iz = 0) const { return z_dim[iz]; }
};
#ifndef ARDUINO
#include "Ekf.cpp"
#endif
#endif
