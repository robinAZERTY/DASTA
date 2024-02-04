#ifndef EKF_HPP
#define EKF_HPP

/*
The Ekf class implements the Extended Kalman Filter (EKF), a recursive filter used for estimating the state of a dynamic system from noisy measurements. This updated version introduces a more streamlined and flexible approach to EKF implementation compared to the previous version.

Key changes:
- Improved modularity: The class now uses templates and function pointers for greater flexibility in defining state transition and measurement functions.
- Reduced memory usage: Temporary matrices and vectors are allocated dynamically as needed, optimizing memory consumption.
- Simplified interface: The class provides template-based methods for prediction, measurement, and update steps, allowing for easy customization and extension.

functional methods:
- predict(): Propagates the state using the provided state transition function and command input, along with the specified process noise covariance matrix.
- mahalanobis(): Computes the Mahalanobis distance between a measurement and the predicted measurement, along with its associated innovation covariance matrix. This method supports flexible measurement functions and covariance matrices.
- update(): Updates the state based on the measurement, using the predicted measurement, its associated innovation covariance matrix, and optionally, the measurement Jacobian and its inverse covariance matrix.

This version aims for improved readability, flexibility, and efficiency compared to the previous implementation.

Next steps:
- Integration with other filtering techniques such as the Unscented Kalman Filter (UKF) for comparison and performance evaluation.
*/


#include "LdlMatrix.hpp"

class Ekf
{
    public :
        template <typename T>
        void jacobian(Matrix &res, T f, Vector &x, const Vector &y0, const data_type h=1e-4);
    
        static Matrix tmp1, tmp2; // temporary Matrix
        static LdlMatrix tmpLdl; // temporary SymMatrix
        static SymMatrix tmpSym;
        static Vector prev_x, y; // previous state
        static Matrix K; // Kalman gain

        void realocTmp1(uint_fast8_t size);

    public:
        Ekf(){};
        Ekf(const uint_fast8_t x_dim){setXDim(x_dim);}
        ~Ekf();

        void setXDim(const uint_fast8_t x_dim);
        Vector x; // state
        data_type x_dim; // state dimension
        SymMatrix P; // state covariance

        template <typename T>
        void predict(T f, Vector &u, const SymMatrix &Q);

        template <typename T>
        void predictMeasurment(T h, Vector &h_pred, const SymMatrix &R, SymMatrix &S, Matrix *H = nullptr, SymMatrix *S_inv = nullptr);

        template <typename T>
        const data_type mahalanobis(T h, const Vector &z, const SymMatrix &R, Vector &h_pred, SymMatrix &S, Matrix *H = nullptr, SymMatrix *S_inv = nullptr, const bool use_h_pred = false, const bool use_H = false, const bool use_S_inv=false);

        template <typename T>
        void update(T h, const Vector &z, const SymMatrix &R, Vector &h_pred, SymMatrix &S, Matrix *H = nullptr, SymMatrix *S_inv = nullptr, const bool use_h_pred = false);

};

#ifndef ARDUINO
#include "ekf.cpp"
#endif

#endif // EKF_HPP