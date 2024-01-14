#ifndef VECTOR_HPP
#define VECTOR_HPP

// #include <Arduino.h>

#ifndef ARDUINO
typedef unsigned char uint_fast8_t;
typedef unsigned short uint_fast16_t;
#endif

typedef float data_type;

class Vector
{
protected:
    static unsigned long long alloc_count;

public:
    static unsigned long long access_count;
    static const unsigned long long memory_usage() { return alloc_count; };

    Vector();
    Vector(uint_fast16_t size);
    ~Vector();

    uint_fast16_t size = 0;
    data_type *data = nullptr;

    const data_type &operator()(uint_fast16_t index) const { return this->data[index]; }
    data_type &operator()(uint_fast16_t index) { return this->data[index]; }

    void fill(const data_type value);
};

// copy data : res<-a
void cd(Vector &res, const Vector &a);

// add : res<-a+b
void add(Vector &res, const Vector &a, const Vector &b);

// substract : res<-a-b
void sub(Vector &res, const Vector &a, const Vector &b);

// multiply : res<-a*b(scalar)
void mul(Vector &res, const Vector &a, const data_type b);

#ifndef ARDUINO
#include "vector.cpp"
#endif

#endif