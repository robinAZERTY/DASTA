#include "SymMatrix.hpp"

SymMatrix::SymMatrix(uint_fast8_t order) : Vector(order * (order + 1) / 2)
{
    this->rows = order;
    this->cols = order;

}


void cd(SymMatrix &res, Matrix &a)
{
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
}

void cd(Matrix &res, SymMatrix &a)
{
    uint_fast8_t i, j;
    uint_fast8_t k = 0;
    for (i = 0; i < a.rows; i++)
    {
        for (j = 0; j <= i; j++)
        {
            res.data[i * a.cols + j] = a.data[k];
            res.data[j * a.cols + i] = a.data[k];
            k++;
        }
    }
}