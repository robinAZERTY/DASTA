#include "matrix.hpp"

Matrix::Matrix()
{
    Vector::alloc_count += sizeof(this->rows) + sizeof(this->cols) + sizeof(this->transposed);
}

Matrix::Matrix(uint_fast8_t rows, uint_fast8_t cols) : Vector(rows * cols)
{
    this->rows = rows;
    this->cols = cols;
    Vector::alloc_count += sizeof(this->rows) + sizeof(this->cols) + sizeof(this->transposed); // this->rows, this->cols, this->transposed
}

Matrix::~Matrix()
{
    Vector::alloc_count -= sizeof(this->rows) + sizeof(this->cols) + sizeof(this->transposed);
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
    if (this->transposed)
        return this->data[col * this->rows + row];
    else
        return this->data[row * this->cols + col];
}

void mul(Matrix &res, Matrix &a, Matrix &b)
{
    for (uint_fast8_t i = 0; i < a.rows; i++)
        for (uint_fast8_t j = 0; j < b.cols; j++)
        {
            data_type sum = 0;
            for (uint_fast8_t k = 0; k < a.cols; k++)
                sum += a(i, k) * b(k, j);
            res(i, j) = sum;
        }
}

void mul(Vector &res, Matrix &a, Vector &b)
{
    for (uint_fast8_t i = 0; i < a.rows; i++)
    {
        data_type sum = 0;
        for (uint_fast16_t k = 0; k < a.cols; k++)
            sum += a(i, k) * b.data[k];
        res.data[i] = sum;
    }
}

void cd(Matrix &res, Matrix &a)
{
    for (uint_fast8_t i = 0; i < res.rows; i++)
        for (uint_fast8_t j = 0; j < res.cols; j++)
            res(i, j) = a(i, j);
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

    for (uint_fast8_t i = 0; i < n; i++)
    {
        // Find the maximum pivot (maximum in absolute value)
        uint_fast8_t pivotRow = i;
        data_type maxPivot = fabs(a(i, i));

        for (uint_fast8_t k = i + 1; k < n; k++)
        {
            data_type absVal = fabs(a(k, i));
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
        for (uint_fast8_t j = 0; j < n; j++)
        {
            a(i, j) *= pivot;
            res(i, j) *= pivot;
        }

        // Eliminate non-zero values below and above the pivot
        for (uint_fast8_t j = 0; j < n; j++)
        {
            if (j != i)
            {
                data_type factor = a(j, i);
                for (uint_fast8_t k = 0; k < n; k++)
                {
                    a(j, k) -= factor * a(i, k);
                    res(j, k) -= factor * res(i, k);
                }
            }
        }
    }

    return true; // Matrix 'a' is successfully inverted, and the result is in 'res'
}