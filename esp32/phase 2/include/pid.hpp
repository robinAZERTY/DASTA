/**
 * @brief This file contains the declaration of the Pid class.
 * 
 * The Pid class implements a Proportional-Integral-Derivative (PID) controller.
 * It provides methods to compute the control signal based on the error and time.
 */

#ifndef PID_HPP
#define PID_HPP

#include <Arduino.h>

class Pid
{
public:
    float lastError;                // Last error value
    float integral;                 // Integral term
    float lastDerivative;           // Last derivative term

public:
    float kp;
    float ki ;
    float kd;
    float maxIntegral;
    float timeConstDerFilter;
    float min;
    float max;
    Pid();                                              // Default constructor
    Pid(float kp, float ki, float kd, float maxIntegral = 1e6, float timeConstDerFilter = 0, float min = -1e9, float max = 1e9);   // Constructor with parameters
    float compute(float error, float time = micros() / 1e6);   // Compute the control signal
    float compute_using_external_derivative(float error, float errorDerivative, float time = micros() / 1e6);   // Compute the control signal
    void reset_integrale(){integral = 0; lastDerivative = 0; lastError = 0;}; // Reset the integral term
};

#endif // PID_HPP