#include "vector.hpp"

// count the number of bytes allocated by the class
unsigned long long Vector::alloc_count = sizeof(Vector::alloc_count);

Vector::Vector()
{
    alloc_count += sizeof(*this);
}

Vector::Vector(uint_fast16_t size)
{
    this->size = size;
    this->data = new data_type[size];
    alloc_count += sizeof(data_type) * size;
    alloc_count += sizeof(*this);
}

Vector::~Vector()
{
    delete[] this->data;
    alloc_count -= sizeof(data_type) * this->size;
    alloc_count -= sizeof(*this);
}

void Vector::set_zero()
{
    for (int i = 0; i < this->size; i++)
        this->data[i] = 0;
}

void cd(Vector &res, Vector &a)
{
    for (int i = 0; i < a.size; i++)
        res.data[i] = a.data[i];
}

void add(Vector &res, Vector &a, Vector &b)
{
    for (int i = 0; i < a.size; i++)
        res.data[i] = a.data[i] + b.data[i];
}

void sub(Vector &res, Vector &a, Vector &b)
{
    for (int i = 0; i < a.size; i++)
        res.data[i] = a.data[i] - b.data[i];
}

void mul(Vector &res, Vector &a, data_type b)
{
    for (int i = 0; i < a.size; i++)
        res.data[i] = a.data[i] * b;
}