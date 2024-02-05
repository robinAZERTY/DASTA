#include "matrix.hpp"

Matrix::Matrix() : Vector()
{
#ifdef SPY
    spy.func_call++;
#endif
}

Matrix::Matrix(uint_fast8_t rows, uint_fast8_t cols)
{
    this->alloc(rows, cols);
#ifdef SPY
    spy.func_call++;
#endif
}

void Matrix::alloc(uint_fast8_t rows, uint_fast8_t cols)
{
#ifdef SPY
    spy.func_call++;
    spy.mul++;
    spy.affect += 2;
#endif

    Vector::alloc(rows * cols);
    this->rows = rows;
    this->cols = cols;
}

Matrix::Matrix(Matrix &m) : Vector(m.size)
{
#ifdef SPY
    spy.func_call++;
    spy.affect += 3;
#endif

    this->rows = m.rows;
    this->cols = m.cols;
    this->transposed = m.transposed;
    vector::cd(*this, m);
}

void Matrix::set_eye()
{
#ifdef SPY
    spy.func_call++;
    uint_fast8_t i, j;
    spy.declaration += 2;

    for (i = 0; i < this->rows; i++)
    {
        spy.add++;
        spy.affect++;
        spy.bool_op += 2;
        for (j = 0; j < this->cols; j++)
        {
            spy.add += 2;
            spy.affect += 2;
            spy.bool_op += 3;
            spy.mul++;

            if (i == j)
                this->data[i * this->cols + j] = 1;
            else
                this->data[i * this->cols + j] = 0;
        }
    }
#else
    uint_fast8_t i, j;
    for (i = 0; i < this->rows; i++)
        for (j = 0; j < this->cols; j++)
            if (i == j)
                this->data[i * this->cols + j] = 1;
            else
                this->data[i * this->cols + j] = 0;
#endif
}

void Matrix::transpose()
{
    this->transposed = !this->transposed;
    uint_fast8_t tmp = this->rows;
    this->rows = this->cols;
    this->cols = tmp;

#ifdef SPY
    spy.func_call++;
    spy.declaration++;
    spy.affect += 4;
    spy.bool_op++;
#endif
}

data_type &Matrix::operator()(uint_fast8_t row, uint_fast8_t col)
{
#ifdef SPY
    spy.func_call++;
    spy.bool_op += 1;
    spy.add++;
    spy.mul++;
    spy.access++;
#endif

    if (this->transposed)
        return this->data[col * this->rows + row];
    else
        return this->data[row * this->cols + col];
}

const data_type &Matrix::operator()(uint_fast8_t row, uint_fast8_t col) const
{
#ifdef SPY
    spy.func_call++;
    spy.cast++;
#endif

    return const_cast<Matrix *>(this)->operator()(row, col);
}
void matrix::mul(Matrix &res, const Matrix &a, const Matrix &b)
{
#ifdef SPY
    spy.func_call++;
    spy.declaration += 13;
    spy.bool_op += 9;

    uint_fast8_t i, j, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    uint_fast8_t &bk = (b.transposed) ? j : k, &bj = (b.transposed) ? k : j, bc = (b.transposed) ? b.rows : b.cols;
    uint_fast8_t &ri = (res.transposed) ? j : i, &rj = (res.transposed) ? i : j, rc = (res.transposed) ? res.rows : res.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
    {
        spy.add++;
        spy.affect++;
        spy.bool_op += 2;
        for (j = 0; j < b.cols; j++)
        {
            spy.add += 2;
            spy.affect += 3;
            spy.bool_op += 2;
            spy.mul++;
            spy.access++;

            sum = 0;
            for (k = 0; k < a.cols; k++)
            {
                spy.add += 4;
                spy.mul += 3;
                spy.affect += 2;
                spy.bool_op += 2;
                spy.access += 2;
                sum += a.data[ai * ac + ak] * b.data[bk * bc + bj];
            }

            res.data[ri * rc + rj] = sum;
        }
    }
#else
    uint_fast8_t i, j, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    uint_fast8_t &bk = (b.transposed) ? j : k, &bj = (b.transposed) ? k : j, bc = (b.transposed) ? b.rows : b.cols;
    uint_fast8_t &ri = (res.transposed) ? j : i, &rj = (res.transposed) ? i : j, rc = (res.transposed) ? res.rows : res.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
        for (j = 0; j < b.cols; j++)
        {
            sum = 0;
            for (k = 0; k < a.cols; k++)
                sum += a.data[ai * ac + ak] * b.data[bk * bc + bj];

            res.data[ri * rc + rj] = sum;
        }
#endif
}

void matrix::mul(Vector &res, const Matrix &a, const Vector &b)
{
#ifdef SPY
    spy.func_call++;
    spy.declaration += 5;
    uint_fast8_t i, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
    {
        spy.add++;
        spy.affect += 3;
        spy.bool_op += 2;
        spy.access++;

        sum = 0;
        for (k = 0; k < a.cols; k++)
        {
            spy.add += 3;
            spy.affect += 2;
            spy.bool_op += 2;
            spy.access += 2;
            spy.mul++;
            sum += a.data[ai * ac + ak] * b.data[k];
        }
        res.data[i] = sum;
    }
#else
    uint_fast8_t i, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
    {
        sum = 0;
        for (k = 0; k < a.cols; k++)
        {
            sum += a.data[ai * ac + ak] * b.data[k];
        }
        res.data[i] = sum;
    }
#endif
}

void matrix::mul_add(Vector &res, const Matrix &a, const Vector &b)
{
#ifdef SPY
    spy.func_call++;
    spy.declaration += 5;
    uint_fast8_t i, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
    {
        spy.add++;
        spy.affect += 3;
        spy.bool_op += 2;
        spy.access++;

        sum = 0;
        for (k = 0; k < a.cols; k++)
        {
            spy.add += 3;
            spy.affect += 2;
            spy.bool_op += 2;
            spy.access += 2;
            spy.mul++;
            sum += a.data[ai * ac + ak] * b.data[k];
        }
        res.data[i] += sum;
    }
#else
    uint_fast8_t i, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    data_type sum;
    for (i = 0; i < a.rows; i++)
    {
        sum = 0;
        for (k = 0; k < a.cols; k++)
        {
            sum += a.data[ai * ac + ak] * b.data[k];
        }
        res.data[i] += sum;
    }
#endif
}

void matrix::refd(Matrix &res, const Matrix &a)
{
    res.data = a.data;
    res.rows = a.rows;
    res.cols = a.cols;
    res.size = a.size;
    res.transposed = a.transposed;

#ifdef SPY
    spy.func_call++;
    spy.affect += 5;
#endif
}

bool matrix::inv(Matrix &res, Matrix &a)
{
#ifdef SPY
    spy.func_call++;
    spy.bool_op += 2;
    if (a.rows == 1)
    {
        spy.div++;
        spy.affect++;
        spy.access += 2;
        res.data[0] = 1 / a.data[0];
        return true;
    }
    spy.bool_op += 2;
    if (a.rows == 2)
    {
        spy.mul += 2;
        spy.sub++;
        spy.affect++;
        spy.access += 4;
        data_type d = a.data[0] * a.data[3] - a.data[1] * a.data[2];
        spy.bool_op += 2;
        spy.func_call++;
        if (fabs(d) < 1e-6)
            return false;

        spy.div++;
        spy.affect++;
        d = 1 / d;
        res.data[0] = a.data[3] * d;
        spy.affect++;
        res.data[1] = -a.data[1] * d;
        spy.affect++;
        spy.sub++;
        res.data[2] = -a.data[2] * d;
        spy.affect++;
        spy.sub++;
        res.data[3] = a.data[0] * d;
        spy.affect++;
        return true;
    }
    // Gauss-Jordan
    // Warning: this function will modify 'a'
    res.set_eye(); // Initialize 'res' as the identity matrix

    spy.declaration += 8;
    uint_fast8_t i, j, k;
    uint_fast8_t pivotRow, ic, jc;
    data_type factor, maxPivot;

    spy.affect++;
    for (i = 0; i < a.rows; i++)
    {
        spy.add++;
        spy.affect += 1;
        spy.bool_op += 2;

        // Find the maximum pivot (maximum in absolute value)
        pivotRow = i;
        spy.affect++;
        maxPivot = fabs(a.data[i * a.cols + i]);
        spy.affect++;
        spy.mul++;
        spy.add++;
        spy.access++;
        spy.func_call++;

        for (k = i + 1; k < a.rows; k++)
        {
            spy.add++;
            spy.affect += 1;
            spy.bool_op += 2;
            factor = fabs(a.data[k * a.cols + i]);
            spy.affect++;
            spy.mul++;
            spy.add++;
            spy.access++;
            spy.func_call++;
            if (factor > maxPivot)
            {
                spy.bool_op += 2;
                maxPivot = factor;
                spy.affect++;
                pivotRow = k;
                spy.affect++;
            }
        }

        // Check if the matrix is singular
        spy.bool_op += 2;
        if (maxPivot < 1e-6)
            return false; // Matrix is singular or nearly singular (zero pivot)

        // Swap rows in both matrices
        ic = i * a.cols;
        spy.affect++;
        spy.mul++;
        jc = pivotRow * a.cols;
        spy.affect++;
        spy.mul++;

        for (k = 0; k < a.cols; k++)
        {
            spy.add++;
            spy.affect++;
            spy.bool_op += 2;
            factor = a.data[ic + k];
            spy.affect++;
            spy.access++;
            spy.add++;
            a.data[ic + k] = a.data[jc + k];
            spy.affect++;
            spy.access += 2;
            spy.add += 2;
            a.data[jc + k] = factor;
            spy.affect++;
            spy.access++;
            spy.add++;
            factor = res.data[ic + k];
            spy.affect++;
            spy.access++;
            spy.add++;
            res.data[ic + k] = res.data[jc + k];
            spy.affect++;
            spy.access += 2;
            spy.add += 2;
            res.data[jc + k] = factor;
            spy.affect++;
            spy.access++;
            spy.add++;
        }

        // Make the diagonal element 1
        factor = 1 / a.data[i * a.cols + i];
        spy.div++;
        spy.affect++;
        spy.access++;
        spy.add++;
        for (j = 0; j < a.cols; j++)
        {
            spy.add++;
            spy.affect++;
            spy.bool_op += 2;
            a.data[ic + j] *= factor;
            spy.access++;
            spy.mul++;
            spy.affect++;
            spy.add++;
            res.data[ic + j] *= factor;
            spy.access++;
            spy.mul++;
            spy.affect++;
            spy.add++;
        }

        // Eliminate non-zero values below and above the pivot
        for (j = 0; j < a.rows; j++)
        {
            spy.add++;
            spy.affect++;
            spy.bool_op += 2;
            spy.bool_op += 2;
            if (j != i)
            {
                factor = -a.data[j * a.cols + i];
                spy.affect++;
                spy.access++;
                spy.mul++;
                spy.add++;
                spy.sub++;
                jc = j * a.cols;
                spy.affect++;
                spy.mul++;
                for (k = 0; k < a.cols; k++)
                {
                    spy.add++;
                    spy.affect++;
                    spy.bool_op += 2;
                    a.data[jc + k] += factor * a.data[ic + k];
                    spy.access += 2;
                    spy.mul++;
                    spy.add += 3;
                    spy.affect++;
                    res.data[jc + k] += factor * res.data[ic + k];
                    spy.access += 2;
                    spy.mul++;
                    spy.add += 3;
                    spy.affect++;
                }
            }
        }
    }

    return true;
#else
    if (a.rows == 1)
    {
        res.data[0] = 1 / a.data[0];
        return true;
    }
    if (a.rows == 2)
    {
        data_type d = a.data[0] * a.data[3] - a.data[1] * a.data[2];
        if (fabs(d) < 1e-6)
            return false;

        d = 1 / d;
        res.data[0] = a.data[3] * d;
        res.data[1] = -a.data[1] * d;
        res.data[2] = -a.data[2] * d;
        res.data[3] = a.data[0] * d;
        return true;
    }

    // Gauss-Jordan
    // Warning: this function will modify 'a'
    res.set_eye(); // Initialize 'res' as the identity matrix

    uint_fast8_t i, j, k;
    uint_fast8_t pivotRow, ic, jc;
    data_type factor, maxPivot;

    for (i = 0; i < a.rows; i++)
    {
        // Find the maximum pivot (maximum in absolute value)
        pivotRow = i;
        maxPivot = fabs(a.data[i * a.cols + i]);

        for (k = i + 1; k < a.rows; k++)
        {
            factor = fabs(a.data[k * a.cols + i]);
            if (factor > maxPivot)
            {
                maxPivot = factor;
                pivotRow = k;
            }
        }

        // Check if the matrix is singular
        if (maxPivot < 1e-6)
            return false; // Matrix is singular or nearly singular (zero pivot)

        // Swap rows in both matrices
        ic = i * a.cols;
        jc = pivotRow * a.cols;

        for (k = 0; k < a.cols; k++)
        {
            factor = a.data[ic + k];
            a.data[ic + k] = a.data[jc + k];
            a.data[jc + k] = factor;
            factor = res.data[ic + k];
            res.data[ic + k] = res.data[jc + k];
            res.data[jc + k] = factor;
        }

        // Make the diagonal element 1
        factor = 1 / a.data[i * a.cols + i];
        for (j = 0; j < a.cols; j++)
        {
            a.data[ic + j] *= factor;
            res.data[ic + j] *= factor;
        }

        // Eliminate non-zero values below and above the pivot
        for (j = 0; j < a.rows; j++)
            if (j != i)
            {
                factor = -a.data[j * a.cols + i];
                jc = j * a.cols;
                for (k = 0; k < a.cols; k++)
                {
                    a.data[jc + k] += factor * a.data[ic + k];
                    res.data[jc + k] += factor * res.data[ic + k];
                }
            }
    }

    return true;
#endif
}