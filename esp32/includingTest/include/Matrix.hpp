#ifndef LINEAR_ALGEBRA_HPP
#define LINEAR_ALGEBRA_HPP

/*
bibliothèque C/C++ légère conçue pour être utilisée sur des microcontrôleurs tels que Arduino et ESP32.
Elle permet de réaliser des opérations de base sur des matrices, telles que l'addition, la soustraction, la multiplication, la transposition et l'inversion
tout en optimisant l'utilisation des ressources.
*/

#include "vector.hpp"

#ifndef ARDUINO
#include <math.h>
#endif

class Matrix : public Vector
{
public:
    uint_fast8_t rows = 0;
    uint_fast8_t cols = 0;
    bool transposed = false;

    // default constructor
    Matrix();
    Matrix(uint_fast8_t rows, uint_fast8_t cols);
    Matrix(Matrix &m);

    void alloc(uint_fast8_t rows, uint_fast8_t cols);

    // set the main diagonal to 1 and the rest to 0
    void set_eye();

    // Swap the positions of rows and columns
    void transpose();

    // accessor operator
    data_type &operator()(uint_fast8_t row, uint_fast8_t col);
    const data_type &operator()(uint_fast8_t row, uint_fast8_t col) const;
    using Vector::operator(); // to avoid hiding the other operator() of Vector

};

namespace matrix
{
// res<-a*b
void mul(Matrix &res, const Matrix &a, const Matrix &b);
void mul(Vector &res, const Matrix &a, const Vector &b);

// res += a*b
void mul_add(Vector &res, const Matrix &a, const Vector &b);

// res<-a^(-1), this function will modify 'a'
bool inv(Matrix &res, Matrix &a);

// refer data : res <- a; res must be declared with default constructor
void refd(Matrix &res, const Matrix &a);

/*
inverse : res <- a^(-1); a <- eye
returns false if the matrix is not invertible
returns true if the matrix was inverted successfully
*/
bool inv(Matrix &res, Matrix &a);
} // namespace matrix

#ifndef ARDUINO
#include "Matrix.cpp"
#endif

#endif