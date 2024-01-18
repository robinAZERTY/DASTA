#include "matrix.hpp"

Matrix::Matrix(uint_fast8_t rows, uint_fast8_t cols) : Vector(rows * cols)
{
    this->rows = rows;
    this->cols = cols;
}

Matrix::Matrix(Matrix &m) : Vector(m.size)
{
    this->rows = m.rows;
    this->cols = m.cols;
    this->transposed = m.transposed;
    cd(*this, m);
}

void Matrix::set_eye()
{
    for (uint_fast8_t i = 0; i < this->rows; i++)
        for (uint_fast8_t j = 0; j < this->cols; j++)
            if (i == j)
                this->data[i * this->cols + j] = 1;
            else
                this->data[i * this->cols + j] = 0;
}

void Matrix::swap_rows(uint_fast8_t i, uint_fast8_t j)
{
    data_type tmp;
    uint_fast8_t ic = i * this->cols;
    uint_fast8_t jc = j * this->cols;

    for (uint_fast8_t k = 0; k < this->cols; k++)
    {
        tmp = this->data[ic + k];
        this->data[ic + k] = this->data[jc + k];
        this->data[jc + k] = tmp;
    }
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
    this->access_count++;
    if (this->transposed)
        return this->data[col * this->rows + row];
    else
        return this->data[row * this->cols + col];
}


void mul(Matrix &res, Matrix &a, Matrix &b)
{
    uint_fast8_t i, j, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    uint_fast8_t &bk = (b.transposed) ? j : k, &bj = (b.transposed) ? k : j, bc = (b.transposed) ? b.rows : b.cols;
    uint_fast8_t &ri = (res.transposed) ? j : i, &rj = (res.transposed) ? i : j, rc = (res.transposed) ? res.rows : res.cols;

    for (i = 0; i < a.rows; i++)
        for (j = 0; j < b.cols; j++)
        {
            data_type sum = 0;
            for (k = 0; k < a.cols; k++)
                sum += a.data[ai * ac + ak] * b.data[bk * bc + bj];

            res.data[ri * rc + rj] = sum;
        }
}

void mul(Vector &res, Matrix &a, Vector &b)
{
    uint_fast8_t i, k;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, ac = (a.transposed) ? a.rows : a.cols;
    
    for (i = 0; i < a.rows; i++)
    {
        data_type sum = 0;
        for (k = 0; k < a.cols; k++)
        {
            sum += a.data[ai * ac + ak] * b.data[k];
        }
        res.data[i] = sum;
    }
}

void refd(Matrix &res, Matrix &a)
{
    res.data = a.data;
    res.rows = a.rows;
    res.cols = a.cols;
    res.size = a.size;
    res.transposed = a.transposed;
}


bool inv(Matrix &res, Matrix &a)
{
    // Gauss-Jordan
    // Warning: this function will modify 'a'

    // Check if the input matrix 'a' is square
    int n = a.rows;
    if (n != a.cols)
        return false; // Non-square matrices cannot be inverted

    res.set_eye(); // Initialize 'res' as the identity matrix
    
    uint_fast8_t i, k, j;
    uint_fast8_t &ai = (a.transposed) ? k : i, &ak = (a.transposed) ? i : k, &aj = (a.transposed) ? j : k, ac = (a.transposed) ? a.rows : a.cols;
    uint_fast8_t &ri = (res.transposed) ? j : i, &rj = (res.transposed) ? i : j, &rk = (res.transposed) ? k : j, rc = (res.transposed) ? res.rows : res.cols;
    
    for (i = 0; i < n; i++)
    {
        // Find the maximum pivot (maximum in absolute value)
        uint_fast8_t pivotRow = i;
        data_type maxPivot = fabs(a.data[ai * ac + ai]);

        for (k = i + 1; k < n; k++)
        {
            data_type absVal = fabs(a.data[ak * ac + ai]);
            if (absVal > maxPivot)
            {
                maxPivot = absVal;
                pivotRow = k;
            }
        }

        // Check if the matrix is singular
        if (maxPivot < 1e-6)
            return false; // Matrix is singular or nearly singular (zero pivot)

        // Swap rows in both matrices
        a.swap_rows(i, pivotRow);
        res.swap_rows(i, pivotRow);

        // Make the diagonal element 1
        data_type pivot = 1 / a(i, i);
        for (j = 0; j < n; j++)
        {
            a.data[ai * ac + aj] *= pivot;
            res.data[ri * rc + rj] *= pivot;
        }

        // Eliminate non-zero values below and above the pivot
        for (j = 0; j < n; j++)
        {
            if (j != i)
            {
                data_type factor = a.data[aj * ac + ai];
                for (k = 0; k < n; k++)
                {
                    a.data[aj * ac + ak] -= factor * a.data[ai * ac + ak];
                    res.data[rj * rc + rk] -= factor * res.data[ri * rc + rk];
                    a.data[aj * ac + ak] -= factor * a.data[ai * ac + ak];
                }
            }
        }
    }

    return true; // Matrix 'a' is successfully inverted, and the result is in 'res'
}