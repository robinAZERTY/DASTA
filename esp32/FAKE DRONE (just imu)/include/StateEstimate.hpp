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
    static float dt_proprio; // période mesuré d'échantillonnage des capteurs proprio
    Ekf *ekf;
    Vector position;
    Vector velocity;
    Quaternion orientation;
};

#endif // STATEESTIMATE_HPP