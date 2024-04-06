#include "matrix.hpp"

Matrix::Matrix() : Vector()
{
}

Matrix::Matrix(uint_fast8_t rows, uint_fast8_t cols)
{
    this->alloc(rows, cols);
}

void Matrix::alloc(uint_fast8_t rows, uint_fast8_t cols)
{
    Vector::alloc(rows * cols);
    this->rows = rows;
    this->cols = cols;
}

Matrix::Matrix(Matrix &m) : Vector(m.size)
{
    this->rows = m.rows;
    this->cols = m.cols;
    this->transposed = m.transposed;
    vector::cd(*this, m);
}

void Matrix::set_eye()
{
    uint_fast8_t i, j;
    for (i = 0; i < this->rows; i++)
        for (j = 0; j < this->cols; j++)
            if (i == j)
                this->data[i * this->cols + j] = 1;
            else
                this->data[i * this->cols + j] = 0;
}

void Matrix::transpose()
{
    this->transposed = !this->transposed;
    uint_fast8_t tmp = this->rows;
    this->rows = this->cols;
    this->cols = tmp;
}

data_type &Matrix::operator()(uint_fast8_t row, uint_fast8_t col)
{
    if (this->transposed)
        return this->data[col * this->rows + row];
    else
        return this->data[row * this->cols + col];
}

const data_type &Matrix::operator()(uint_fast8_t row, uint_fast8_t col) const
{
    return const_cast<Matrix *>(this)->operator()(row, col);
}
void matrix::mul(Matrix &res, const Matrix &a, const Matrix &b)
{
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
}

void matrix::mul(Vector &res, const Matrix &a, const Vector &b)
{
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
}

void matrix::mul_add(Vector &res, const Matrix &a, const Vector &b)
{
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
}