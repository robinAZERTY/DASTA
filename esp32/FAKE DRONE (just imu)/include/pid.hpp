/**
 * @brief This file contains the declaration of the Pid class.
 * 
 * The Pid class implements a Proportional-Integral-Derivative (PID) controller.
 * It provides methods to compute the control signal based on the error and time.
 */

#ifndef PID_HPP
#define PID_HPP

#include <Arduino.h>
#include "Vector.hpp"

class Pid
{
public:
    float lastError;                // Last error value
    float integral;                 // Integral term
    float lastDerivative;           // Last derivative term
    float lastTime;                 // Last time value
    float deltaTime;                // Time difference between two consecutive calls to compute

public:
    Vector params = Vector(7);      // PID parameters
    // float kp=0, ki=0, kd=0, timeConstDerFilter, maxIntegral=1e9, min=-1e9, max=1e9;   // PID parameters
    float &kp(){return params.data[0];}
    float &ki(){return params.data[1];}
    float &kd(){return params.data[2];}
    float &maxIntegral(){return params.data[3];}
    float &timeConstDerFilter(){return params.data[4];}
    float &min(){return params.data[5];}
    float &max(){return params.data[6];}
    Pid();                                              // Default constructor
    Pid(float kp, float ki, float kd, float maxIntegral = 1e6, float timeConstDerFilter = 0, float min = -1e9, float max = 1e9);   // Constructor with parameters
    float compute(float error, float time = micros() / 1e6);   // Compute the control signal
    void unset() { lastTime = -1; }   // Reset the last time value
};

#endif // PID_HPP