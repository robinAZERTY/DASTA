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
#ifdef SPY
    spy.func_call++;
    spy.bool_op += 2;
#endif
}

void LdlMatrix::alloc_L_D(uint_fast8_t order)
{
#ifdef SPY
    spy.func_call++;
    spy.affect++;
#endif

    if (LD_allocated)
        throw "L and D are already allocated";
    L.alloc(order);
    _D.alloc(order);
    LD_allocated = true;
}

void t_matrix::inv(UTMatrix &res, const UTMatrix &a)
{
#ifndef SPY
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
#else
    spy.func_call++;
    spy.declaration += 4;
    spy.affect++;
    uint_fast8_t i, j, k;
    data_type s;
    uint8_t cse; // common sub expression
    for (i = 1; i < a.order; i++)
    {
        spy.bool_op += 2;
        spy.affect += 2;
        spy.add++;
        cse = (i - 1) * i >> 1;
        spy.affect++;
        spy.sub++;
        spy.mul++;
        spy.bShift++;

        for (j = 0; j < i; j++)
        {
            spy.bool_op += 2;
            spy.affect += 3;
            spy.add += 2;
            s = a.data[cse + j];
            spy.affect++;
            spy.access++;
            spy.add++;
            for (k = j + 1; k < i; k++)
            {
                spy.bool_op += 2;
                spy.affect++;
                spy.add++;
                s += a.data[cse + k] * res.data[((k - 1) * k >> 1) + j];
                spy.affect++;
                spy.access += 2;
                spy.sub++;
                spy.mul += 2;
                spy.bShift++;
                spy.add += 3;
            }
            res.data[cse + j] = -s;
            spy.affect++;
            spy.access++;
            spy.add++;
            spy.sub++;
        }
    }
#endif
}

void t_matrix::mul(TMatrix &res, const UTMatrix &a, const DiagoMatrix &b)
{
#ifndef SPY
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

#else
    spy.func_call++;
    spy.declaration += 6;
    spy.affect += 3;
    spy.bool_op += 4;
    spy.add++;

    res.transposed = a.transposed;
    uint_fast8_t i, j, k;
    uint_fast8_t &bi = (a.transposed) ? i : j;
    uint_fast8_t cse1, cse2; // common sub expression

    for (i = 0; i < a.order; i++)
    {
        spy.bool_op += 2;
        spy.affect += 2;
        spy.add++;
        cse1 = i * (i + 1) >> 1;
        spy.affect++;
        spy.mul++;
        spy.add++;
        spy.bShift++;
        cse2 = (i - 1) * i >> 1;
        spy.affect++;
        spy.sub++;
        spy.add++;
        spy.bShift++;
        for (j = 0; j < i; j++)
        {
            spy.bool_op += 2;
            spy.affect++;
            spy.add += 2;
            res.data[cse1 + j] = b.data[bi] * a.data[cse2 + j];
            spy.affect++;
            spy.access += 3;
            spy.mul++;
            spy.add += 2;
        }
        res.data[cse1 + i] = b.data[i];
        spy.affect++;
        spy.access += 2;
        spy.add++;
    }
#endif
}

void t_matrix::mul(SymMatrix &res, const TMatrix &a, const UTMatrix &b)
{
#ifndef SPY
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
#else
    spy.func_call++;
    spy.bool_op += 4;

    if (!a.transposed || b.transposed)
        throw "the result is not symmetric";

    spy.declaration += 5;
    spy.affect++;
    uint_fast8_t i, j, k;
    data_type s;
    uint_fast8_t cse; // common sub expression

    for (i = 0; i < a.order; i++)
    {
        spy.bool_op += 2;
        spy.affect += 2;
        spy.add++;
        for (j = i; j < a.order; j++)
        {
            spy.bool_op += 2;
            spy.affect += 2;
            spy.add++;
            cse = (j * (j + 1) >> 1) + i;
            s = a.data[cse];
            spy.affect += 2;
            spy.access++;
            spy.add += 2;
            spy.mul++;
            spy.bShift++;
            for (k = j + 1; k < a.order; k++)
            {
                spy.bool_op += 2;
                spy.affect++;
                spy.add++;
                s += a.data[(k * (k + 1) >> 1) + i] * b.data[((k - 1) * k >> 1) + j];
                spy.affect++;
                spy.access += 2;
                spy.mul += 3;
                spy.bShift += 2;
                spy.add += 4;
                spy.sub++;
            }
            res.data[cse] = s;
            spy.affect++;
            spy.access++;
        }
    }
#endif
}

// https://fr.wikipedia.org/wiki/Factorisation_de_Cholesky

TMatrix LdlMatrix::tmpT;

void LdlMatrix::decompose()
{
#ifndef SPY
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
#else
    spy.func_call++;
    spy.declaration += 5;
    spy.affect++;
    spy.bool_op += 2;
    if (!LD_allocated)
        this->alloc_L_D(this->order);
    uint_fast8_t i, j, k;
    uint_fast8_t cse1, cse2; // common sub expression
    for (j = 0; j < this->order; j++)
    {
        spy.bool_op += 2;
        spy.affect += 2;
        spy.add++;
        _D.data[j] = this->data[(j * (j + 1) >> 1) + j];
        spy.affect++;
        spy.access += 2;
        spy.add += 2;
        spy.mul++;
        spy.bShift++;
        cse2 = (j - 1) * j >> 1;
        spy.affect++;
        spy.sub++;
        spy.mul++;
        spy.bShift++;

        for (k = 0; k < j; k++)
        {
            spy.bool_op += 2;
            spy.affect++;
            spy.add++;
            _D.data[j] -= L.data[cse2 + k] * L.data[cse2 + k] * _D.data[k];
            spy.affect++;
            spy.access += 4;
            spy.mul += 2;
            spy.add += 2;
            spy.sub++;
        }

        for (i = j + 1; i < this->order; i++)
        {
            cse1 = (i - 1) * i >> 1;
            spy.affect++;
            spy.sub++;
            spy.mul++;
            spy.bShift++;
            spy.bool_op += 2;
            spy.affect += 2;
            spy.add++;
            L.data[cse1 + j] = this->data[(i * (i + 1) >> 1) + j];
            spy.affect++;
            spy.access += 2;
            spy.add += 3;
            spy.mul++;
            spy.bShift++;
            for (k = 0; k < j; k++)
            {
                spy.bool_op += 2;
                spy.affect++;
                spy.add++;
                L.data[cse1 + j] -= L.data[cse1 + k] * L.data[cse2 + k] * _D.data[k];
                spy.affect++;
                spy.access += 4;
                spy.mul += 2;
                spy.add += 3;
                spy.sub++;
            }
            L.data[cse1 + j] /= _D.data[j];
            spy.affect++;
            spy.access += 2;
            spy.div++;
            spy.add++;
        }
    }
#endif
}

void ldl_matrix::det(data_type &res, LdlMatrix &a)
{
#ifndef SPY

    if (sym_matrix::det(res, a))
        return;

    if (!a.decomposed)
        a.decompose();
    diag_matrix::det(res, a._D);

#else
    spy.func_call++;
    spy.bool_op += 2;
    if (sym_matrix::det(res, a))
        return;

    spy.bool_op += 2;
    if (!a.decomposed)
        a.decompose();
    diag_matrix::det(res, a._D);

#endif
}

void ldl_matrix::inv(SymMatrix &res, LdlMatrix &a, bool force_det)
{
#ifndef SPY
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

#else
    spy.func_call++;
    spy.bool_op += 2;
    if (sym_matrix::inv(res, a))
        return;

    spy.bool_op += 2;
    if (!a.decomposed)
        a.decompose();
    t_matrix::inv(a.L, a.L);
    diag_matrix::inv(a._D, a._D);

    spy.affect += 5;
    a.L.transposed = true;
    LdlMatrix::tmpT.order = a.order;
    LdlMatrix::tmpT.data = a.data;
    t_matrix::mul(LdlMatrix::tmpT, a.L, a._D);
    a.L.transposed = false;
    t_matrix::mul(res, LdlMatrix::tmpT, a.L);
    LdlMatrix::tmpT.data = nullptr;
#endif
}