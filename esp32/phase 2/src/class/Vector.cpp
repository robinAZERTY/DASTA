#include "vector.hpp"

// count the number of bytes allocated by the class
unsigned long long Vector::alloc_count = 0;

data_type Vector::_zero = 0;
data_type Vector::_one = 1;

Vector::Vector()
{
}

Vector::Vector(uint_fast16_t size)
{
    this->alloc(size);
}

const unsigned long long Vector::memory_usage()
{
    return alloc_count;
}

void Vector::alloc(uint_fast16_t size)
{
    if (this->data != nullptr)
    {
        delete[] this->data;
        alloc_count -= sizeof(data_type) * this->size;
    }

    this->size = size;
    this->data = new data_type[size];
    alloc_count += sizeof(data_type) * size;

}

Vector::~Vector()
{
    if (this->data != nullptr)
    {
        delete[] this->data;
        alloc_count -= sizeof(data_type) * this->size;
    }

}

void Vector::fill(const data_type value)
{
    for (uint_fast16_t i = 0; i < this->size; i++)
        this->data[i] = value;
}

data_type &Vector::operator()(uint_fast16_t i)
{
    return this->data[i];
}

const data_type &Vector::operator()(uint_fast16_t i) const
{
    return const_cast<Vector *>(this)->operator()(i);
}

void vector::cd(Vector &res, const Vector &a)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i];

}

void vector::add(Vector &res, const Vector &a, const Vector &b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i] + b.data[i];
}

void vector::sub(Vector &res, const Vector &a, const Vector &b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i] - b.data[i];
}

void vector::mul(Vector &res, const Vector &a, const data_type b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i] * b;

}

void vector::mul(data_type &res, const Vector &a, const Vector &b)
{
    res = 0;
    for (uint_fast8_t i = 0; i < a.size; i++)
        res += a.data[i] * b.data[i];
}

bool vector::eq(const Vector &a, const Vector &b)
{
    if (a.size != b.size)
        return false;

    for (uint_fast16_t i = 0; i < a.size; i++)
        if (a.data[i] != b.data[i])
            return false;

    return true;
}

bool vector::eq(const Vector &a, const data_type b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        if (a.data[i] != b)
            return false;

    return true;
}