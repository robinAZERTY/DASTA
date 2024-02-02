#include "DiagoMatrix.hpp"


data_type &DiagoMatrix::operator()(uint_fast8_t row, uint_fast8_t col)
{
    if (row == col)
        return this->data[row];
    else
        return Vector::_zero;
}

void diag_matrix::mul(Matrix &res, const Matrix &a, const DiagoMatrix &b)
{
    uint_fast8_t i, j;
    uint_fast8_t ac = (a.transposed) ? a.rows : a.cols;
    uint_fast8_t &ri = (res.transposed) ? j : i, &rj = (res.transposed) ? i : j, rc = (res.transposed) ? res.rows : res.cols;

    for (i = 0; i < a.rows; i++)
        for (j = 0; j < b.size; j++)
            res.data[ri * rc + rj] = a.data[i * ac + j] * b.data[j];
}


void diag_matrix::add(Matrix &res, const DiagoMatrix &a, const Matrix &b)
{
    if (&res != &b)
        for (uint_fast8_t i = 0; i < res.size; i++)
            res.data[i] = b.data[i];

    uint_fast8_t rc = (res.transposed) ? res.rows : res.cols;
    for (uint_fast8_t i = 0; i < rc; i++)
        res.data[i+rc*i] += a.data[i];
}

void diag_matrix::sub(Matrix &res, const DiagoMatrix &a, const Matrix &b)
{
    for (uint_fast8_t i = 0; i < res.size; i++)
        res.data[i] = -b.data[i];

    uint_fast8_t rc = (res.transposed) ? res.rows : res.cols;
    for (uint_fast8_t i = 0; i < rc; i++)
        res.data[i+rc*i] += a.data[i];
}

void diag_matrix::inv(DiagoMatrix &res, const DiagoMatrix &a)
{
    #ifndef SPY
    for (uint_fast8_t i = 0; i < res.size; i++)
        res.data[i] = 1 / a.data[i];
    #else
    spy.func_call++;
    spy.declaration ++;
    spy.affect++;
    for (uint_fast8_t i = 0; i < res.size; i++)
    {
        spy.bool_op+=2;
        spy.affect++;
        res.data[i] = 1 / a.data[i];spy.affect++;spy.access+=2;spy.div++;
    }
    #endif
}

void diag_matrix::det(data_type &res, const DiagoMatrix &a)
{
    res = 1;
    for (uint_fast8_t i = 0; i < a.size; i++)
        res *= a.data[i];
}