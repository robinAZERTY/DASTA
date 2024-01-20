#ifndef SYM_MATRIX_HPP
#define SYM_MATRIX_HPP

    //the data is stored in a lower triangular matrix in row major order like this:
    // matrix:
    // 0 1 3
    // 1 2 4
    // 3 4 5
    // matrix in memory:
    // -> 0 1 2 3 4 5

#include "Matrix.hpp"

#ifndef ARDUINO
#include <math.h>
#endif

class SymMatrix : public Vector
{
    public:
        uint_fast8_t rows = 0;
        uint_fast8_t cols = 0;

        // default constructor
        SymMatrix():Vector(){};
        SymMatrix(uint_fast8_t order);

};

// res<-a
void cd(SymMatrix &res, Matrix &a);
void cd(Matrix &res, SymMatrix &a);


#endif