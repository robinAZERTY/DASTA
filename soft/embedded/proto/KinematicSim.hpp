/*
class to predict IMU measurments of the quadcopter from motors commands only
assuming 
*/
#include "Vector.hpp"

class KinematicSim
{
private:
    float m, g, l, k, b, s; // mass, gravity, arm length, propeler thrust coefficient, propeler drag coefficient, time response constant of the motor
    float Ixx, Iyy, Izz; // inertia
    float cmd1, cmd2, cmd3, cmd4;
    float w1, w2, w3, w4; // motor speed

    Vector accel = Vector(3);
    Vector angulVel = Vector(3);

public:
    KinematicSim();
    ~KinematicSim();
    // setters
    void setCmd(float cmd1, float cmd2, float cmd3, float cmd4){this->cmd1 = cmd1; this->cmd2 = cmd2; this->cmd3 = cmd3; this->cmd4 = cmd4;}
    void setMass(float m){this->m = m;}
    void setGravity(float g){this->g = g;}
    void setArmLength(float l){this->l = l;}
    void setPropelerThrustCoefficient(float k){this->k = k;}
    void setPropelerDragCoefficient(float b){this->b = b;}
    void setMotorTimeConstant(float s){this->s = s;}
    void setInertiaX(float Ixx){this->Ixx = Ixx;}
    void setInertiaY(float Iyy){this->Iyy = Iyy;}
    void setInertiaZ(float Izz){this->Izz = Izz;}

    // update
    void predict(float dt);
    
    // getters
    Vector &getAccel(){return accel;}
    Vector &getAngulVel(){return angulVel;}
};