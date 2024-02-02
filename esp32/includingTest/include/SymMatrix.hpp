#ifndef SYM_MATRIX_HPP
#define SYM_MATRIX_HPP

// the data is stored in a lower triangular matrix in row major order like this:
//  matrix:
//  0 1 3
//  1 2 4
//  3 4 5
//  in memory:
//  -> 0 1 2 3 4 5

#include "Matrix.hpp"

#ifndef ARDUINO
#include <math.h>
#endif

class SymMatrix : public Vector
{
public:
    uint_fast8_t order = 0;
    data_type _det = nan("");
    // default constructor
    SymMatrix() : Vector(){};
    SymMatrix(uint_fast8_t order);
    virtual void alloc(uint_fast8_t order);

    data_type &operator()(uint_fast8_t row, uint_fast8_t col);
    const data_type &operator()(uint_fast8_t row, uint_fast8_t col) const;
    
    using Vector::operator(); // to avoid hiding the other operator() of Vector
};

namespace sym_matrix
{
    // res<-a
    void cd(SymMatrix &res, Matrix &a);
    void cd(Matrix &res, SymMatrix &a);

    void add(SymMatrix &res, const SymMatrix &a, const Matrix &b);
    void add(Matrix &res, const SymMatrix &a, const Matrix &b);

    // res<-a*b
    void mul(Matrix &res, const Matrix &a, const SymMatrix &b);
    void mul(Matrix &res, const SymMatrix &a, const Matrix &b);
    void mul(SymMatrix &res, const Matrix &a, const Matrix &b);
    void mul(SymMatrix &res, const Matrix &a, const SymMatrix &b);
    void mul(Vector &res, const SymMatrix &a, const Vector &b);


    /*
    res<-det(a)
    direct calculation of the determinant for small matrices
    returns false if the size of the matrix is too big
    returns true if the determinant was calculated successfully
    */
    bool det(data_type &res, SymMatrix &a, bool force = false);

    /*
    res<-inv(a)
    direct calculation of the inverse for small matrices
    returns -1 if the matrix is not invertible
    returns 0 if the size of the matrix is too big
    returns 1 if the matrix was inverted successfully
    */
    int inv(SymMatrix &res, SymMatrix &a, bool force_det = false);

}

#ifndef ARDUINO
#include "SymMatrix.cpp"
#endif

#endif