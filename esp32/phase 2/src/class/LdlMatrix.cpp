#include "LdlMatrix.hpp"

data_type &TMatrix::operator()(uint_fast8_t row, uint_fast8_t col)
{
    if (transposed)
    {
        uint_fast8_t tmp = row;
        row = col;
        col = tmp;
    }
    if (row >= col)
        return SymMatrix::operator()(row, col);
    else
        return Vector::_zero;
}

const data_type &TMatrix::operator()(uint_fast8_t row, uint_fast8_t col) const
{
    return const_cast<TMatrix *>(this)->operator()(row, col);
}

data_type &UTMatrix::operator()(uint_fast8_t row, uint_fast8_t col)
{
    if (transposed)
    {
        uint_fast8_t tmp = row;
        row = col;
        col = tmp;
    }

    if (row > col)
    {
        return SymMatrix::operator()(row - 1, col);
    }
    else if (row == col)
        return Vector::_one;
    else
        return Vector::_zero;
}

const data_type &UTMatrix::operator()(uint_fast8_t row, uint_fast8_t col) const
{
    return const_cast<UTMatrix *>(this)->operator()(row, col);
}

void LdlMatrix::alloc(uint_fast8_t order)
{
    SymMatrix::alloc(order);
    if (order > 2)
        alloc_L_D(order);
}

void LdlMatrix::alloc_L_D(uint_fast8_t order)
{

    if (LD_allocated)
        throw "L and D are already allocated";
    L.alloc(order);
    _D.alloc(order);
    LD_allocated = true;
}

void t_matrix::inv(UTMatrix &res, const UTMatrix &a)
{
    uint_fast8_t i, j, k;
    data_type s;
    uint8_t cse; // common sub expression
    for (i = 1; i < a.order; i++)
    {
        cse = (i - 1) * i >> 1;
        for (j = 0; j < i; j++)
        {
            s = a.data[cse + j];
            for (k = j + 1; k < i; k++)
                s += a.data[cse + k] * res.data[((k - 1) * k >> 1) + j];
            res.data[cse + j] = -s;
        }
    }
}

void t_matrix::mul(TMatrix &res, const UTMatrix &a, const DiagoMatrix &b)
{
    res.transposed = a.transposed;
    uint_fast8_t i, j, k;
    uint_fast8_t &bi = (a.transposed) ? i : j;
    uint_fast8_t cse1, cse2; // common sub expression
    for (i = 0; i < a.order; i++)
    {
        cse1 = i * (i + 1) >> 1;
        cse2 = (i - 1) * i >> 1;
        for (j = 0; j < i; j++)
            res.data[cse1 + j] = b.data[bi] * a.data[cse2 + j];
        res.data[cse1 + i] = b.data[i];
    }
}

void t_matrix::mul(SymMatrix &res, const TMatrix &a, const UTMatrix &b)
{
    if (!a.transposed || b.transposed)
        throw "the result is not symmetric";

    uint_fast8_t i, j, k;
    data_type s;
    uint_fast8_t cse; // common sub expression
    for (i = 0; i < a.order; i++)
    {
        for (j = i; j < a.order; j++)
        {
            cse = (j * (j + 1) >> 1) + i;
            s = a.data[cse];
            for (k = j + 1; k < a.order; k++)
                s += a.data[(k * (k + 1) >> 1) + i] * b.data[((k - 1) * k >> 1) + j];
            res.data[cse] = s;
        }
    }
}

// https://fr.wikipedia.org/wiki/Factorisation_de_Cholesky

TMatrix LdlMatrix::tmpT;

void LdlMatrix::decompose()
{
    if (!LD_allocated)
        this->alloc_L_D(this->order);

    uint_fast8_t i, j, k;
    uint_fast8_t cse1, cse2; // common sub expression
    for (j = 0; j < this->order; j++)
    {
        _D.data[j] = this->data[(j * (j + 1) >> 1) + j];
        cse2 = (j - 1) * j >> 1;
        for (k = 0; k < j; k++)
            _D.data[j] -= L.data[cse2 + k] * L.data[cse2 + k] * _D.data[k];

        for (i = j + 1; i < this->order; i++)
        {
            cse1 = (i - 1) * i >> 1;
            L.data[cse1 + j] = this->data[(i * (i + 1) >> 1) + j];
            for (k = 0; k < j; k++)
            {
                L.data[cse1 + j] -= L.data[cse1 + k] * L.data[cse2 + k] * _D.data[k];
            }
            L.data[cse1 + j] /= _D.data[j];
        }
    }
}

void ldl_matrix::det(data_type &res, LdlMatrix &a)
{

    if (sym_matrix::det(res, a))
        return;

    if (!a.decomposed)
        a.decompose();
    diag_matrix::det(res, a._D);
}

void ldl_matrix::inv(SymMatrix &res, LdlMatrix &a, bool force_det)
{
    if (sym_matrix::inv(res, a, force_det))
        return;

    if (!a.decomposed)
        a.decompose();
    t_matrix::inv(a.L, a.L);
    diag_matrix::inv(a._D, a._D);

    a.L.transposed = true;
    LdlMatrix::tmpT.order = a.order;
    LdlMatrix::tmpT.data = a.data;
    t_matrix::mul(LdlMatrix::tmpT, a.L, a._D);
    a.L.transposed = false;
    t_matrix::mul(res, LdlMatrix::tmpT, a.L);
    LdlMatrix::tmpT.data = nullptr;
}