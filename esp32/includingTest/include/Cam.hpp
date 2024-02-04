#ifndef CAM_HPP
#define CAM_HPP

#include "SymMatrix.hpp"
#include "Quaternion.hpp"
#include "Led.hpp"

class Cam
{
public:
    Cam();
    ~Cam();

    void project(Vector res, const Vector &led_pos, const Quaternion &drone_orien, const Vector &drone_pos);

    Quaternion orientation;
    Vector position;
    data_type k;// constant in pixel*m/m

    //for the ekf
    SymMatrix noise_cov;
    // for every led
    Vector *led_predictions;
    SymMatrix *led_predictions_cov;
    SymMatrix *led_predictions_cov_inv;
    Matrix *led_predictions_jac;

    Vector *measurments;// les position we read from the cameras
    uint_fast8_t measurments_num=0;
    uint_fast8_t measurments_alloc=0;
};

#endif // CAM_HPP
