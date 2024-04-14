#ifndef STATEESTIMATE_HPP
#define STATEESTIMATE_HPP
#include "ekf.hpp"
#include "quaternion.hpp"
#include "lowPassFilter.hpp"
class StateEstimate
{
public:

    bool running = true;

    StateEstimate();
    ~StateEstimate();
    void run(const float dt);
    Ekf ekf;
    data_type *dt_proprio;
    data_type *gravity;

    Quaternion orientation = Quaternion(nullptr);
    data_type get_orientation_max_cov();
    Vector rp_offset = Vector(2);
    void set_rp_offset();
    Vector rpy = Vector(3);
    Vector rpy_leveled = Vector(3);
    Vector velocity = Vector(3);
    Vector position = Vector(3);
    void initFromAccel(const Vector &accel);

    Vector last_gyro = Vector(3);
    LowPassFilter lpf_gyr_derx, lpf_gyr_dery, lpf_gyr_derz;
};

#endif // STATEESTIMATE_HPP