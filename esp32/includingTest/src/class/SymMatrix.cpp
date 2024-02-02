#include "SymMatrix.hpp"

SymMatrix::SymMatrix(uint_fast8_t order) : Vector((order * (order + 1)) >> 1)
{
#ifdef SPY
    spy.func_call++;
    spy.affect++;
    spy.bShift++;
    spy.mul++;
    spy.add++;
#endif

    this->order = order;
}

void SymMatrix::alloc(uint_fast8_t order)
{
#ifdef SPY
    spy.func_call++;
    spy.affect++;
    spy.mul++;
    spy.add++;
    spy.bShift++;
#endif

    Vector::alloc((order * (order + 1)) >> 1);
    this->order = order;
}

data_type &SymMatrix::operator()(uint_fast8_t row, uint_fast8_t col)
{
#ifdef SPY
    spy.func_call++;
    spy.mul++;
    spy.add += 2;
    spy.bShift++;
    spy.bool_op += 2;
    spy.access++;
#endif
    if (row >= col)
        return this->data[(row * (row + 1) >> 1) + col];
    else
        return this->data[(col * (col + 1) >> 1) + row];
}

const data_type &SymMatrix::operator()(uint_fast8_t row, uint_fast8_t col) const
{
#ifdef SPY
    spy.func_call++;
    spy.cast++;
#endif
    return const_cast<SymMatrix *>(this)->operator()(row, col);
}

void sym_matrix::cd(SymMatrix &res, Matrix &a)
{
#ifdef SPY
    spy.func_call++;
    spy.affect += 2;
    spy.declaration += 3;
    uint_fast8_t i, j;
    uint_fast8_t k = 0;
    for (i = 0; i < a.rows; i++)
    {
        spy.bool_op += 2;
        spy.add++;
        spy.affect += 2;

        for (j = 0; j <= i; j++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect++;
            res.data[k] = a.data[i * a.cols + j];
            spy.mul++;
            spy.add++;
            spy.access += 2;
            spy.affect++;
            k++;
            spy.add++;
            spy.affect++;
        }
    }
#else
    uint_fast8_t i, j;
    uint_fast8_t k = 0;
    for (i = 0; i < a.rows; i++)
    {
        for (j = 0; j <= i; j++)
        {
            res.data[k] = a.data[i * a.cols + j];
            k++;
        }
    }
#endif
}

void sym_matrix::cd(Matrix &res, SymMatrix &a)
{
#ifndef SPY
    uint_fast8_t i, j;
    uint_fast8_t k = 0, ai;
    for (i = 0; i < a.order; i++)
    {
        ai = a.order * i;
        for (j = 0; j <= i; j++)
        {
            res.data[ai + j] = a.data[k];
            res.data[j * a.order + i] = a.data[k];
            k++;
        }
    }
#else
    spy.func_call++;
    spy.affect += 2;
    spy.declaration += 4;
    uint_fast8_t i, j;
    uint_fast8_t k = 0, ai;
    for (i = 0; i < a.order; i++)
    {
        ai = a.order * i;
        spy.bool_op += 2;
        spy.add++;
        spy.affect += 2;
        spy.mul++;
        for (j = 0; j <= i; j++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect++;
            res.data[ai + j] = a.data[k];
            spy.access += 2;
            spy.add++;
            spy.affect++;
            res.data[j * a.order + i] = a.data[k];
            spy.access += 2;
            spy.affect++;
            spy.add++;
            spy.mul++;
            k++;
            spy.add++;
            spy.affect++;
        }
    }
#endif
}

void sym_matrix::add(SymMatrix &res, const SymMatrix &a, const Matrix &b)
{
#ifndef SPY
    uint_fast8_t i, j;
    uint_fast8_t k = 0;
    for (i = 0; i < a.order; i++)
    {
        for (j = 0; j <= i; j++)
        {
            res.data[k] = a.data[k] + b.data[i * a.order + j];
            k++;
        }
    }
#else
    spy.func_call++;
    spy.affect += 2;
    spy.declaration += 3;
    uint_fast8_t i, j;
    uint_fast8_t k = 0;
    for (i = 0; i < a.order; i++)
    {
        spy.bool_op += 2;
        spy.add++;
        spy.affect += 2;
        for (j = 0; j <= i; j++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect++;
            res.data[k] = a.data[k] + b.data[i * a.order + j];
            spy.access += 3;
            spy.add += 2;
            spy.mul++;
            spy.affect++;
            k++;
            spy.add++;
            spy.affect++;
        }
    }
#endif
}

void sym_matrix::add(Matrix &res, const SymMatrix &a, const Matrix &b)
{
#ifndef SPY
    uint_fast8_t i, j;
    uint_fast8_t k = 0, ai;
    for (i = 0; i < a.order; i++)
    {
        ai = a.order * i;
        for (j = 0; j <= i; j++)
        {
            res.data[ai + j] = a.data[k] + b.data[ai + j];
            res.data[j * a.order + i] = a.data[k] + b.data[ai + j];
            k++;
        }
    }
#else
    spy.func_call++;
    spy.affect += 2;
    spy.declaration += 4;
    uint_fast8_t i, j;
    uint_fast8_t k = 0, ai;
    for (i = 0; i < a.order; i++)
    {
        spy.bool_op += 2;
        spy.add++;
        spy.affect += 2;
        ai = a.order * i;
        for (j = 0; j <= i; j++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect++;
            res.data[ai + j] = a.data[k] + b.data[ai + j];
            spy.access += 3;
            spy.add += 3;
            spy.affect++;
            res.data[j * a.order + i] = a.data[k] + b.data[ai + j];
            spy.access += 3;
            spy.add += 2;
            spy.mul++;
            spy.affect++;
            k++;
            spy.add++;
            spy.affect++;
        }
    }
#endif
}

void sym_matrix::mul(Matrix &res, const Matrix &a, const SymMatrix &b)
{
#ifndef SPY
    uint_fast8_t i, j, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    uint_fast8_t &ri = (res.transposed) ? j : i, &rj = (res.transposed) ? i : j, rc = (res.transposed) ? res.rows : res.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
        for (j = 0; j < b.order; j++)
        {
            sum = 0;
            for (k = 0; k < j; k++)
                sum += a.data[ai * ac + ak] * b.data[(j * (j + 1) >> 1) + k];
            for (k = j; k < a.cols; k++)
                sum += a.data[ai * ac + ak] * b.data[(k * (k + 1) >> 1) + j];
            res.data[ri * rc + rj] = sum;
        }
#else
    spy.func_call++;
    spy.affect += 7;
    spy.declaration += 9;
    uint_fast8_t i, j, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    uint_fast8_t &ri = (res.transposed) ? j : i, &rj = (res.transposed) ? i : j, rc = (res.transposed) ? res.rows : res.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
    {
        spy.bool_op += 2;
        spy.add++;
        spy.affect += 2;
        for (j = 0; j < b.order; j++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect++;
            sum = 0;
            for (k = 0; k < j; k++)
            {
                spy.bool_op += 2;
                spy.add++;
                spy.affect++;
                sum += a.data[ai * ac + ak] * b.data[(j * (j + 1) >> 1) + k];
                spy.access += 2;
                spy.add += 4;
                spy.mul += 2;
                spy.affect++;
                spy.bShift++;
            }
            for (k = j; k < a.cols; k++)
            {
                spy.bool_op += 2;
                spy.add++;
                spy.affect++;
                sum += a.data[ai * ac + ak] * b.data[(k * (k + 1) >> 1) + j];
                spy.access += 2;
                spy.add += 4;
                spy.mul += 2;
                spy.affect++;
                spy.bShift++;
            }
            res.data[ri * rc + rj] = sum;
            spy.access++;
            spy.affect++;
            spy.add++;
            spy.mul++;
            spy.affect++;
        }
    }
#endif
}

void sym_matrix::mul(Matrix &res, const SymMatrix &a, const Matrix &b)
{
#ifndef SPY
    uint_fast8_t i, j, k;
    uint_fast8_t &bk = (b.transposed) ? j : k, &bj = (b.transposed) ? k : j, bc = (b.transposed) ? b.rows : b.cols;
    uint_fast8_t &ri = (res.transposed) ? j : i, &rj = (res.transposed) ? i : j, rc = (res.transposed) ? res.rows : res.cols;
    data_type sum;
    for (i = 0; i < a.order; i++)
        for (j = 0; j < b.cols; j++)
        {
            sum = 0;
            for (k = 0; k < i; k++)
                sum += a.data[(i * (i + 1) >> 1) + k] * b.data[bk * bc + bj];
            for (k = i; k < b.rows; k++)
                sum += a.data[(k * (k + 1) >> 1) + i] * b.data[bk * bc + bj];
            res.data[ri * rc + rj] = sum;
        }
#else
    spy.func_call++;
    spy.affect += 7;
    spy.declaration += 9;
    uint_fast8_t i, j, k;
    uint_fast8_t &bk = (b.transposed) ? j : k, &bj = (b.transposed) ? k : j, bc = (b.transposed) ? b.rows : b.cols;
    uint_fast8_t &ri = (res.transposed) ? j : i, &rj = (res.transposed) ? i : j, rc = (res.transposed) ? res.rows : res.cols;
    data_type sum;
    for (i = 0; i < a.order; i++)
    {
        spy.bool_op += 2;
        spy.add++;
        spy.affect += 2;
        for (j = 0; j < b.cols; j++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect += 4;
            sum = 0;
            for (k = 0; k < i; k++)
            {
                spy.bool_op += 2;
                spy.add++;
                spy.affect++;
                sum += a.data[(i * (i + 1) >> 1) + k] * b.data[bk * bc + bj];
                spy.access += 2;
                spy.add += 4;
                spy.mul += 2;
                spy.affect++;
                spy.bShift++;
            }
            for (k = i; k < b.rows; k++)
            {
                spy.bool_op += 2;
                spy.add++;
                spy.affect++;
                sum += a.data[(k * (k + 1) >> 1) + i] * b.data[bk * bc + bj];
                spy.access += 2;
                spy.add += 4;
                spy.mul += 2;
                spy.affect++;
                spy.bShift++;
            }
            res.data[ri * rc + rj] = sum;
            spy.access++;
            spy.affect++;
            spy.add++;
            spy.mul++;
        }
    }
#endif
}

void sym_matrix::mul(SymMatrix &res, const Matrix &a, const Matrix &b)
{
#ifndef SPY
    uint_fast8_t i, j, k, l = 0;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    uint_fast8_t &bk = (b.transposed) ? j : k, &bj = (b.transposed) ? k : j, bc = (b.transposed) ? b.rows : b.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
        for (j = 0; j <= i; j++)
        {
            sum = 0;
            for (k = 0; k < a.cols; k++)
                sum += a.data[ai * ac + ak] * b.data[bk * bc + bj];
            res.data[l] = sum;
            l++;
        }
#else
    spy.func_call++;
    spy.affect += 8;
    spy.declaration += 11;
    uint_fast8_t i, j, k, l = 0;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    uint_fast8_t &bk = (b.transposed) ? j : k, &bj = (b.transposed) ? k : j, bc = (b.transposed) ? b.rows : b.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
    {
        spy.bool_op += 2;
        spy.add++;
        spy.affect += 2;
        for (j = 0; j <= i; j++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect += 3;
            sum = 0;
            for (k = 0; k < a.cols; k++)
            {
                spy.bool_op += 2;
                spy.add++;
                spy.affect++;
                sum += a.data[ai * ac + ak] * b.data[bk * bc + bj];
                spy.access += 2;
                spy.add += 3;
                spy.mul += 3;
                spy.affect++;
            }
            res.data[l] = sum;
            spy.access++;
            spy.affect++;
            l++;
            spy.add++;
            spy.affect++;
        }
    }
#endif
}

void sym_matrix::mul(SymMatrix &res, const Matrix &a, const SymMatrix &b)
{
#ifndef SPY
    uint_fast8_t i, j, k, l = 0;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
        for (j = 0; j <= i; j++)
        {
            sum = 0;
            for (k = 0; k < j; k++)
                sum += a.data[ai * ac + ak] * b.data[(j * (j + 1) >> 1) + k];
            for (k = j; k < a.cols; k++)
                sum += a.data[ai * ac + ak] * b.data[(k * (k + 1) >> 1) + j];
            res.data[l] = sum;
            l++;
        }
#else
    spy.func_call++;
    spy.affect += 5;
    spy.declaration += 9;
    uint_fast8_t i, j, k, l = 0;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
    {
        spy.bool_op += 2;
        spy.add++;
        spy.affect += 2;
        for (j = 0; j <= i; j++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect += 4;
            sum = 0;
            for (k = 0; k < j; k++)
            {
                spy.bool_op += 2;
                spy.add++;
                spy.affect++;
                sum += a.data[ai * ac + ak] * b.data[(j * (j + 1) >> 1) + k];
                spy.access += 2;
                spy.add += 3;
                spy.mul += 3;
                spy.bShift++;
                spy.affect++;
            }
            for (k = j; k < a.cols; k++)
            {
                spy.bool_op += 2;
                spy.add++;
                spy.affect++;
                sum += a.data[ai * ac + ak] * b.data[(k * (k + 1) >> 1) + j];
                spy.access += 2;
                spy.add += 3;
                spy.mul += 3;
                spy.bShift++;
                spy.affect++;
            }
            res.data[l] = sum;
            spy.access++;
            spy.affect++;
            l++;
            spy.add++;
            spy.affect++;
        }
    }
#endif
}

bool sym_matrix::det(data_type &res, SymMatrix &a, bool force)
{
#ifndef SPY
    if (!isnan(a._det) && !force)
    {
        res = a._det;
        return true;
    }
    else if (a.order == 1)
    {
        res = a.data[0];
        a._det = res;
        return true;
    }
    else if (a.order == 2)
    {
        res = a.data[0] * a.data[2] - a.data[1] * a.data[1];
        a._det = res;
        return true;
    }
    else
        return false;
#else
    spy.func_call += 2;
    spy.bool_op += 4;
    if (!isnan(a._det) && !force)
    {
        spy.affect++;
        res = a._det;
        a._det = res;
        spy.affect++;
        return true;
    }
    else if (a.order == 1)
    {
        spy.bool_op += 2;
        res = a.data[0];
        a._det = res;
        spy.affect++;

        spy.access++;
        spy.affect++;
        return true;
    }
    else if (a.order == 2)
    {
        spy.bool_op += 2;
        res = a.data[0] * a.data[2] - a.data[1] * a.data[1];
        a._det = res;
        spy.affect++;
        spy.access += 4;
        spy.affect++;
        spy.mul += 2;
        return true;
    }
    else
        return false;
#endif
}

int sym_matrix::inv(SymMatrix &res, SymMatrix &a, bool force_det)
{
#ifndef SPY
    if (a.order == 1)
    {
        if (a.data[0] == 0)
            return -1;
        res.data[0] = 1 / a.data[0];
        return 1;
    }
    else if (a.order == 2)
    {
        data_type det;
        if (sym_matrix::det(det, a, force_det) < 0)
            return 0;
        if (det == 0)
            return -1;
        det = 1 / det;
        res.data[0] = a.data[2] * det;
        res.data[1] = -a.data[1] * det;
        res.data[2] = a.data[0] * det;
        return 1;
    }
    else
        return 0;
#else
    spy.func_call++;
    spy.bool_op += 2;
    if (a.order == 1)
    {
        spy.bool_op += 2;
        spy.access++;
        if (a.data[0] == 0)
            return -1;
        spy.affect++;
        spy.div++;
        spy.access += 2;
        res.data[0] = 1 / a.data[0];
        return 1;
    }
    else
    {
        spy.bool_op += 2;
        if (a.order == 2)
        {
            spy.declaration++;
            spy.bool_op += 2;
            data_type det = a.data[0] * a.data[2] - a.data[1] * a.data[1];
            spy.access += 4;
            spy.affect++;
            spy.mul += 2;
            spy.sub++;
            if (det == 0)
                return -1;
            det = 1 / det;
            spy.affect++;
            spy.div++;
            res.data[0] = a.data[2] * det;
            spy.access += 2;
            spy.affect++;
            spy.mul++;
            res.data[1] = -a.data[1] * det;
            spy.access += 2;
            spy.affect++;
            spy.mul++;
            ;
            spy.sub++;
            res.data[2] = a.data[0] * det;
            spy.access += 2;
            spy.affect++;
            spy.mul++;
            return 1;
        }
        else
            return 0;
    }
#endif
}

void sym_matrix::mul(Vector &res, const SymMatrix &a, const Vector &b)
{
#ifndef SPY
    uint_fast8_t i, k;
    data_type sum;
    for (i = 0; i < b.size; i++)
    {
        sum = 0;
        for (k = 0; k < i; k++)
            sum += a.data[(i * (i + 1) >> 1) + k] * b.data[k];
        for (k = i; k < a.order; k++)
            sum += a.data[(k * (k + 1) >> 1) + i] * b.data[k];
        res.data[i] = sum;
    }
#else
    spy.func_call++;
    spy.affect++;
    spy.declaration += 3;
    uint_fast8_t i, k;
    data_type sum;
    for (i = 0; i < a.order; i++)
    {
        spy.bool_op += 2;
        spy.add++;
        spy.affect += 2;

        sum = 0;
        for (k = 0; k < i; k++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect++;
            sum += a.data[(i * (i + 1) >> 1) + k] * b.data[k];
            spy.access += 2;
            spy.add += 3;
            spy.mul += 2;
            spy.affect++;
            spy.bShift++;
        }
        for (k = i; k < b.size; k++)
        {
            spy.bool_op += 2;
            spy.add++;
            spy.affect++;
            sum += a.data[(k * (k + 1) >> 1) + i] * b.data[k];
            spy.access += 2;
            spy.add += 3;
            spy.mul += 2;
            spy.affect++;
            spy.bShift++;
        }

        res.data[i] = sum;
        spy.access++;
        spy.affect++;
    }
#endif
}