#include "Vector.hpp"

// count the number of bytes allocated by the class
unsigned long long Vector::alloc_count = 0;
unsigned long long Vector::access_count = 0;

Vector::Vector(uint_fast16_t size)
{
    this->size = size;
    this->data = new data_type[size];
    alloc_count += sizeof(data_type) * size;
}

Vector::~Vector()
{
    delete[] this->data;
    alloc_count -= sizeof(data_type) * this->size;
}

void Vector::fill(const data_type value)
{
    for (uint_fast16_t i = 0; i < this->size; i++)
        this->data[i] = value;
}

void cd(Vector &res, const Vector &a)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i];
}

void cd(data_type *res, const Vector &a)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res[i] = a.data[i];
}

void add(Vector &res, const Vector &a, const Vector &b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i] + b.data[i];
}

void sub(Vector &res, const Vector &a, const Vector &b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i] - b.data[i];
}

void mul(Vector &res, const Vector &a, const data_type b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i] * b;
}
