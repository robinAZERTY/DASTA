#ifndef STATEESTIMATE_HPP
#define STATEESTIMATE_HPP
#include "Ekf.hpp"
#include "Quaternion.hpp"

class StateEstimate
{
private:
    float last_time_extero=-1;

public:
    float last_time_proprio=-1;
    data_type gravity;
    bool running = false;
    StateEstimate();
    ~StateEstimate();
    float dt_proprio; // période mesuré d'échantillonnage des capteurs proprio
    Vector command;
    SymMatrix command_cov;
    Ekf ekf;
    Vector position;
    Vector velocity;
    Quaternion orientation;
};

#endif // STATEESTIMATE_HPP