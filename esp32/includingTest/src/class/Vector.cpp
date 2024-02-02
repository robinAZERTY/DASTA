#include "vector.hpp"

// count the number of bytes allocated by the class
unsigned long long Vector::alloc_count = 0;

data_type Vector::_zero = 0;
data_type Vector::_one = 1;

Vector::Vector()
{
#ifdef SPY
    spy.func_call++;
#endif
}

Vector::Vector(uint_fast16_t size)
{
    this->alloc(size);
#ifdef SPY
    spy.func_call++;
#endif
}

const unsigned long long Vector::memory_usage()
{
#ifdef SPY
    spy.func_call++;
#endif
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

#ifdef SPY
    spy.func_call++;
    spy.bool_op++;
    spy.bool_op++;
    if (this->data != nullptr)
    {
        spy.mul++;
        spy.sub++;
        spy.affect++;
        delete[] this->data;
        alloc_count -= sizeof(data_type) * this->size;
        spy.func_call++;
    }
    spy.affect++;
    spy.alloc += sizeof(data_type) * size;
    spy.affect++;
    spy.mul++;
    spy.affect++;
    this->size = size;
    this->data = new data_type[size];
    alloc_count += sizeof(data_type) * size;
#endif
}

Vector::~Vector()
{
    if (this->data != nullptr)
    {
        delete[] this->data;
        alloc_count -= sizeof(data_type) * this->size;
    }

#ifdef SPY
    spy.func_call++;
    spy.mul++;
    spy.sub++;
    spy.affect++;
#endif
}

void Vector::fill(const data_type value)
{
    for (uint_fast16_t i = 0; i < this->size; i++)
        this->data[i] = value;

#ifdef SPY
    spy.func_call++;
    spy.bool_op += this->size;
    spy.bool_op += this->size;
    spy.add += this->size;
    spy.access += this->size;
    spy.affect += 2 * this->size;
#endif
}

data_type &Vector::operator()(uint_fast16_t i)
{
#ifdef SPY
    spy.func_call++;
    spy.access++;
#endif

    return this->data[i];
}

const data_type &Vector::operator()(uint_fast16_t i) const
{
#ifdef SPY
    spy.func_call++;
    spy.cast++;
#endif

    return const_cast<Vector *>(this)->operator()(i);
}

void vector::cd(Vector &res, const Vector &a)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i];

#ifdef SPY
    spy.func_call++;
    spy.declaration++;
    spy.bool_op += a.size;
    spy.bool_op += a.size;
    spy.add += a.size;
    spy.access += 2 * a.size;
    spy.affect += 2 * a.size;
#endif
}

void vector::add(Vector &res, const Vector &a, const Vector &b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i] + b.data[i];

#ifdef SPY
    spy.func_call++;
    spy.declaration++;
    spy.bool_op += a.size;
    spy.bool_op += a.size;
    spy.add += 2 * a.size;
    spy.access += 3 * a.size;
    spy.affect += 2 * a.size;
#endif
}

void vector::sub(Vector &res, const Vector &a, const Vector &b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i] - b.data[i];

#ifdef SPY
    spy.func_call++;
    spy.declaration++;
    spy.bool_op += a.size;
    spy.bool_op += a.size;
    spy.add += a.size;
    spy.sub += a.size;
    spy.access += 3 * a.size;
    spy.affect += 2 * a.size;
#endif
}

void vector::mul(Vector &res, const Vector &a, const data_type b)
{
    for (uint_fast16_t i = 0; i < a.size; i++)
        res.data[i] = a.data[i] * b;

#ifdef SPY
    spy.func_call++;
    spy.declaration++;
    spy.bool_op += a.size;
    spy.bool_op += a.size;
    spy.add += a.size;
    spy.mul += a.size;
    spy.access += 2 * a.size;
    spy.affect += 2 * a.size;
#endif
}

void vector::mul(data_type &res, const Vector &a, const Vector &b)
{
    res = 0;
    for (uint_fast8_t i = 0; i < a.size; i++)
        res += a.data[i] * b.data[i];

#ifdef SPY
    spy.func_call++;
    spy.declaration++;
    spy.bool_op += a.size;
    spy.bool_op += a.size;
    spy.add += a.size;
    spy.mul += a.size;
    spy.access += 3 * a.size;
    spy.affect += 2 * a.size + 1;

#endif
}