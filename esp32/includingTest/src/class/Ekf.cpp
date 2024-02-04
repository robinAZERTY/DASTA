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
    // cout << " J = " << tmp1 << endl;
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
void Ekf::predictMeasurment(T h, Vector &h_pred, const SymMatrix &R, SymMatrix &S, Matrix *H, SymMatrix *S_inv)
{
    h(h_pred, x);
    jacobian(tmp1, h, x, h_pred);

    if (H != nullptr)
    {
        const uint_fast8_t tmp1_size = tmp1.size;
        tmp1.size = h_pred.size * x_dim;
        vector::cd(*H, tmp1);
        tmp1.size = tmp1_size;
    }

    tmp2.rows = x_dim;
    tmp2.cols = z.size;
    tmp1.transpose();
    sym_matrix::mul(tmp2, P, tmp1);
    tmp1.transpose();
    sym_matrix::mul(S, tmp1, tmp2);
    vector::add(S, S, R);

    if (z.size > y.size)
        y.alloc(z.size);

    vector::sub(y, z, h_pred);

    if (tmpLdl.size < S.size)
        tmpLdl.alloc(S.order);

    vector::cd(tmpLdl, S); // copy S to tmpLdl to save the original S

    uint_fast8_t tmp1_size = tmp1.size;
    tmp1.size = z.size;

    if (S_inv != nullptr)
    {
        ldl_matrix::inv(*S_inv, tmpLdl);
        sym_matrix::mul(tmp1, *S_inv, y);
    }
    else
    {
        tmpSym.order = S.order;
        ldl_matrix::inv(tmpSym, tmpLdl);
        sym_matrix::mul(tmp1, tmpSym, y);
    }
}

template <typename T>
const data_type Ekf::mahalanobis(T h, const Vector &z, const SymMatrix &R, Vector &h_pred, SymMatrix &S, Matrix *H, SymMatrix *S_inv, const bool use_h_pred, const bool use_H, const bool use_S_inv)
{
    if (!use_h_pred)
        h(h_pred, x);

    if (use_H and H != nullptr)
        vector::cd(tmp1, *H);
    else
        jacobian(tmp1, h, x, h_pred);

    if (H != nullptr)
    {
        const uint_fast8_t tmp1_size = tmp1.size;
        tmp1.size = z.size * x_dim;
        vector::cd(*H, tmp1);
        tmp1.size = tmp1_size;
    }

    tmp2.rows = x_dim;
    tmp2.cols = z.size;
    tmp1.transpose();
    sym_matrix::mul(tmp2, P, tmp1);
    tmp1.transpose();
    sym_matrix::mul(S, tmp1, tmp2);
    vector::add(S, S, R);

    if (z.size > y.size)
        y.alloc(z.size);

    vector::sub(y, z, h_pred);

    if (!use_S_inv)
    {
        if (tmpLdl.size < S.size)
            tmpLdl.alloc(S.order);

        vector::cd(tmpLdl, S); // copy S to tmpLdl to save the original S

        uint_fast8_t tmp1_size = tmp1.size;
        tmp1.size = z.size;

        if (S_inv != nullptr)
        {
            ldl_matrix::inv(*S_inv, tmpLdl);
            sym_matrix::mul(tmp1, *S_inv, y);
        }
        else
        {
            tmpSym.order = S.order;
            ldl_matrix::inv(tmpSym, tmpLdl);
            sym_matrix::mul(tmp1, tmpSym, y);
        }
    }
    else
    {
        sym_matrix::mul(tmp1, *S_inv, y);
    }
    data_type res;
    vector::mul(res, y, tmp1);
    tmp1.size = tmp1_size;

    return res;
}

template <typename T>
void Ekf::update(T h, const Vector &z, const SymMatrix &R, Vector &h_pred, SymMatrix &S, Matrix *H, SymMatrix *S_inv, const bool use_h_pred)
{
    if (tmp1.size < z.size * x_dim)
        realocTmp1(z.size * x_dim);

    if (tmp2.size < x_dim * z.size)
        tmp2.Vector::alloc(x_dim * z.size);

    vector::cd(prev_x, x);

    if (!use_h_pred)
        h(h_pred, x);

    if (H != nullptr)
    {
        tmp1.rows = z.size;
        tmp1.cols = x_dim;
        vector::cd(tmp1, *H);
    }
    else
        jacobian(tmp1, h, x, h_pred);

    if (z.size > y.size)
        y.alloc(z.size);

    vector::sub(y, z, h_pred);
    tmp2.rows = x_dim;
    tmp2.cols = z.size;
    tmp1.transpose();
    sym_matrix::mul(tmp2, P, tmp1);
    tmp1.transpose();

    if (K.size < x_dim * z.size)
        K.alloc(x_dim, z.size);

    K.rows = x_dim;
    K.cols = z.size;

    if (S_inv == nullptr)
    {
        sym_matrix::mul(S, tmp1, tmp2);
        vector::add(S, S, R);
        tmpSym.order = S.order;

        if (tmpLdl.size < S.size)
            tmpLdl.alloc(S.order);

        tmpSym.order = S.order;
        vector::cd(tmpLdl, S); // copy S to tmpLdl to save the original S

        ldl_matrix::inv(tmpSym, tmpLdl);
        sym_matrix::mul(K, tmp2, tmpSym);
    }
    else
        sym_matrix::mul(K, tmp2, *S_inv);

    matrix::mul(x, K, y);
    vector::add(x, x, prev_x);
    tmp2.transpose();
    sym_matrix::sub_mul(P, K, tmp2);
    tmp2.transpose();
}