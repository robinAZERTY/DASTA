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
        Matrix():Vector(){};
        Matrix(uint_fast8_t rows, uint_fast8_t cols);
        Matrix(Matrix &m);

        void set_eye();
        void swap_rows(uint_fast8_t i, uint_fast8_t j);
        void transpose();

        // accessor operator
        data_type &operator()(uint_fast8_t row, uint_fast8_t col);
};

// res<-a*b
void mul(Matrix &res, Matrix &a, Matrix &b);
void mul(Vector &res, Matrix &a, Vector &b);


// res<-a^(-1) ; a<-a^(-1)
bool inv(Matrix &res, Matrix &a);

// refer data : res <- a; res must be declared with default constructor
void refd(Matrix &res, Matrix &a);

// inverse : res <- a^(-1); a <- eye
bool inv(Matrix &res, Matrix &a);


#ifndef ARDUINO
#include "matrix.cpp"
#endif

#endif