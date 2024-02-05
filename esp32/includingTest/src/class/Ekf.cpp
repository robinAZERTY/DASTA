#include "ekf.hpp"

Matrix Ekf::tmp1, Ekf::tmp2; // temporary Matrix
Vector Ekf::prev_x;          // previous state
SymMatrix Ekf::tmpSym;       // temporary SymMatrix
Vector Ekf::y;               // temporary Vector
Matrix Ekf::K;               // Kalman gain
LdlMatrix Ekf::tmpLdl;       // temporary SymMatrix

template <typename T>
void Ekf::jacobian(Matrix &res, T f, Vector &x, const Vector &y0, const data_type h)
{
    // make sure the size of the result is correct (not protected against bad allocation)
    res.rows = y0.size;
    res.cols = x.size;

    const data_type h_inv = 1 / h;
    uint_fast8_t i, j;
    for (i = 0; i < x.size; i++)
    {
        x.data[i] += h;
        f(tmp2, x);
        x.data[i] -= h;
        for (j = 0; j < y0.size; j++)
            res.data[j * x.size + i] = (tmp2.data[j] - y0.data[j]) * h_inv;
    }
}

void Ekf::realocTmp1(const uint_fast8_t size)
{
    tmp1.Vector::alloc(size);
    tmpSym.data = tmp1.data;
}

void Ekf::setXDim(const uint_fast8_t x_dim)
{
    this->x_dim = x_dim;
    x.alloc(x_dim);
    prev_x.alloc(x_dim);
    P.alloc(x_dim);
    tmp1.alloc(x_dim, x_dim);
    tmp2.alloc(x_dim, x_dim);
    tmpSym.data = tmp1.data;
}

Ekf::~Ekf()
{
    // unshare memory before deallocating (to avoid delete nullptr)
    tmpSym.data = nullptr;
}

template <typename T>
void Ekf::predict(T f, Vector &u, const SymMatrix &Q)
{
    vector::cd(prev_x, x);
    // predict the state
    f(x, prev_x, u);

    auto fx = [&u, &f](Matrix &res, Vector &x)
    { f(res, x, u); };
    // predict the covariance
    jacobian(tmp1, fx, prev_x, x);

    tmp2.rows = x_dim;
    tmp2.cols = x_dim;
    sym_matrix::mul(tmp2, tmp1, P);
    tmp1.transpose(); // tmp& <- J^T
    sym_matrix::mul(P, tmp2, tmp1);
    tmp1.transpose();

    auto fu = [this, &f](Vector &res, const Vector &u)
    { f(res, prev_x, u); };

    jacobian(tmp1, fu, u, x);
    tmp2.rows = x_dim;
    tmp2.cols = u.size;
    sym_matrix::mul(tmp2, tmp1, Q);
    tmp1.transpose();
    sym_matrix::add_mul(P, tmp2, tmp1);
    tmp1.transpose();
}

template <typename T>
void Ekf::predictMeasurment(T h, const SymMatrix &R, Vector &h_pred, SymMatrix &S_inv, Matrix &PH_t)
{
    //predict the measurement
    h(h_pred, x);

    //compute the jacobian
    jacobian(tmp1, h, x, h_pred);

    //compute P*H^T
    tmp1.transpose();
    sym_matrix::mul(PH_t, P, tmp1);
    tmp1.transpose();

    //prepare the S matrix (measurment predition covariance matrix)
    if (tmpLdl.size < R.size)
        tmpLdl.alloc(R.order);

    //compute S
    sym_matrix::mul(tmpLdl, tmp1, PH_t);
    vector::add(tmpLdl, tmpLdl, R);

    //compute S^(-1)
    ldl_matrix::inv(S_inv, tmpLdl);
}

const data_type Ekf::mahalanobis(const Vector &h_pred, const SymMatrix &S_inv, const Vector &z)
{
    if (y.size < z.size)
        y.alloc(z.size);

    y.size = z.size;
    vector::sub(y, z, h_pred);

    const uint_fast8_t tmp1_size = tmp1.size;
    tmp1.size = h_pred.size * x_dim;
    sym_matrix::mul(tmp1, S_inv, y);

    data_type res;
    vector::mul(res, y, tmp1);
    tmp1.size = tmp1_size;

    return res;
}


void Ekf::update(const Vector &z, const Vector &h_pred, const SymMatrix &S_inv, Matrix &PH_t)
{
    //prepare the y vector
    if (z.size > y.size)
        y.alloc(z.size);
    y.size = z.size;

    //compute the inovation
    vector::sub(y, z, h_pred);

    //prepare the K matrix
    if (K.size < x_dim * z.size)
        K.alloc(x_dim, z.size);
    K.rows = x_dim;
    K.cols = z.size;

    //compute the Kalman gain
    sym_matrix::mul(K, PH_t, S_inv);

    //update the state
    matrix::mul_add(x, K, y);

    //update the covariance
    PH_t.transpose();
    sym_matrix::sub_mul(P, K, PH_t);
    PH_t.transpose();
}