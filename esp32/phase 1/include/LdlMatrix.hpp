#ifndef LDL_MATRIX_HPP
#define LDL_MATRIX_HPP

/*
compute LDL decomposition of a symmetric matrix (A = L * D * L^T)
with L lower triangular (ones on the diagonal) and D diagonal.
To compute the determinant of A, multiply the diagonal elements of D.
A^-1 = (L * D * L^T)^-1 = (L^T)^-1 * D^-1 * L^-1 = L * D^-1 * L^T
*/

#include "diagoMatrix.hpp"
#include "SymMatrix.hpp"
/*
triangular matrix
in matrix form:
1 0 0 0
2 3 0 0
4 5 6 0
7 8 9 1
in memory:
-> 1 2 4 7 3 5 8 9 1
*/
class TMatrix : public SymMatrix
{
public:
    bool transposed = false;

    // default constructor
    TMatrix() : SymMatrix(){};
    TMatrix(uint_fast8_t order) { this->alloc(order); };

    void alloc(uint_fast8_t order)
    {
        SymMatrix::alloc(order);
        this->order = order;
    }

    void transpose() { transposed = !transposed; }
    // accessor operator
    data_type &operator()(uint_fast8_t row, uint_fast8_t col);
    const data_type &operator()(uint_fast8_t row, uint_fast8_t col) const;
    using Vector::operator(); // to avoid hiding the other operator() of Vector
};

/* unit triangular matrix with ones on the diagonal (to not save the ones)
in matrix form:
1 0 0 0
1 1 0 0
2 3 1 0
4 5 5 1
in memory:
-> 1 2 3 4 5 6
*/
class UTMatrix : public TMatrix
{
public:
    // default constructor
    UTMatrix() : TMatrix(){};
    UTMatrix(uint_fast8_t order) { this->alloc(order); };

    void alloc(uint_fast8_t order)
    {
        SymMatrix::alloc(order - 1);
        this->order = order;
    }

    // accessor operator
    data_type &operator()(uint_fast8_t row, uint_fast8_t col);
    const data_type &operator()(uint_fast8_t row, uint_fast8_t col) const;
    using Vector::operator(); // to avoid hiding the other operator() of Vector
};

namespace t_matrix
{
    // res<- a^-1
    void inv(UTMatrix &res, const UTMatrix &a);

    // res<-a*b
    void mul(TMatrix &res, const UTMatrix &a, const DiagoMatrix &b);
    void mul(SymMatrix &res, const TMatrix &a, const UTMatrix &b);
}; // namespace lt_matrix

class LdlMatrix : public SymMatrix
{
protected:
    bool LD_allocated = false;

public:
    static TMatrix tmpT;

    bool decomposed = false;
    // default constructor
    LdlMatrix() : SymMatrix(){};
    LdlMatrix(uint_fast8_t order){this->alloc(order);};

    void alloc(uint_fast8_t order);
    void alloc_L_D(uint_fast8_t order);
    void decompose();
    UTMatrix L;
    DiagoMatrix _D;
};

namespace ldl_matrix
{
    // res<-det(a)
    void det(data_type &res, LdlMatrix &a);

    // res<-a^-1 (a.data is destroyed)
    void inv(SymMatrix &res, LdlMatrix &a, bool force_det = true);

}; // namespace ldl_matrix

#ifndef ARDUINO
#include "LdlMatrix.cpp"
#endif

#endif // LDL_MATRIX_HPP