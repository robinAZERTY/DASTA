#ifndef DIAG_MATRIX_HPP
#define DIAG_MATRIX_HPP

#include "matrix.hpp"

class DiagoMatrix : public Vector
{
public:
    // default constructor
    DiagoMatrix() : Vector(){};
    DiagoMatrix(uint_fast8_t size) : Vector(size){};

    // accessor operator
    data_type &operator()(uint_fast8_t row, uint_fast8_t col);
    // data_type &operator()(uint_fast8_t i) const { return this->data[i]; };
    using Vector::operator(); // to avoid hiding the other operator() of Vector
};

namespace diag_matrix
{

// res<-a*b
void mul(Matrix &res, const Matrix &a, const DiagoMatrix &b);

// res<-a+b
void add(Matrix &res, const DiagoMatrix &a, const Matrix &b);

// res<-a-b
void sub(Matrix &res, const DiagoMatrix &a, const Matrix &b);

// res<-a^-1
void inv(DiagoMatrix &res, const DiagoMatrix &a);

// res<-det(a)
void det(data_type &res, const DiagoMatrix &a);
    
} // namespace dia



#ifndef ARDUINO
#include "DiagoMatrix.cpp"
#endif
#endif