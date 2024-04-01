#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <Arduino.h>

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
    static data_type _zero, _one;
    static const unsigned long long memory_usage();

    Vector();
    Vector(uint_fast16_t size);
    virtual ~Vector();

    virtual void alloc(uint_fast16_t size);

    uint_fast16_t size = 0;
    data_type *data = nullptr;

    const data_type &operator()(uint_fast16_t index) const;
    data_type &operator()(uint_fast16_t index);

    void fill(const data_type value);
};

namespace vector
{
    // copy data : res<-a
    void cd(Vector &res, const Vector &a);

    // add : res<-a+b
    void add(Vector &res, const Vector &a, const Vector &b);

    // substract : res<-a-b
    void sub(Vector &res, const Vector &a, const Vector &b);

    // multiply : res<-a*b
    void mul(Vector &res, const Vector &a, const data_type b);
    void mul(data_type &res, const Vector &a, const Vector &b);

} // namespace vector
using namespace vector;

#ifndef ARDUINO
#include "Vector.cpp"
#endif

#endif