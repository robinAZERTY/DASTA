#ifndef STATEESTIMATE_HPP
#define STATEESTIMATE_HPP
#include "ekf.hpp"
#include "quaternion.hpp"

class StateEstimate
{
private:
    float last_time_proprio=-1;
    float last_time_extero=-1;

public:
    bool running = false;
    StateEstimate();
    ~StateEstimate();
    void run(const float time);
    Ekf ekf;
    data_type *dt_proprio;
    data_type *gravity;

    Quaternion orientation = Quaternion(nullptr);
    Vector rpy = Vector(3);
    void initFromAccel(const Vector &accel);
};

#endif // STATEESTIMATE_HPP