// This file contains the implementation of the Pid class, which represents a Proportional-Integral-Derivative controller.

#include "pid.hpp"

// Default constructor
Pid::Pid() : Pid(0, 0, 0, 0, 0, 0, 0)
{
}

// Parameterized constructor
Pid::Pid(float kp, float ki, float kd, float maxIntegral, float timeConstDerFilter, float min, float max)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->maxIntegral = maxIntegral;
    this->timeConstDerFilter = timeConstDerFilter;
    this->min = min;
    this->max = max;
    integral = 0;
    lastDerivative = 0;
    lastError = 0;
}

// Compute the control signal based on the error and current time
float Pid::compute(float error, float dt)
{
    integral += error * dt;

    float integraleComp = ki * integral;
    if (integraleComp > maxIntegral)
        integral = maxIntegral / ki;
    else if (integraleComp < -maxIntegral)
        integral = -maxIntegral / ki;

    // compute the derivative filter
    float derivative = 0;
    if (dt > 1e-6)
        derivative = (error - lastError) / dt;
    lastError = error;

    float alpha = 0;
    if (timeConstDerFilter)
        alpha = timeConstDerFilter / (timeConstDerFilter + dt);
    float filteredDerivative = (1 - alpha) * derivative + alpha * lastDerivative;
    lastDerivative = filteredDerivative;

    float ret = 0;

    if (kp != 0)
        ret += kp * error;

    if (ki != 0)
        ret += ki * integral;

    if (kd != 0 && dt > 0)
        ret += kd * filteredDerivative;

    ret = (ret < min) ? min : ret;
    ret = (ret > max) ? max : ret;

    return ret;
}

float Pid::compute_using_external_derivative(float error, float errorDerivative, float dt)
{
    integral += error * dt;

    float integraleComp = ki * integral;
    if (integraleComp > maxIntegral)
        integral = maxIntegral / ki;
    else if (integraleComp < -maxIntegral)
        integral = -maxIntegral / ki;

    float ret = 0;

    if (kp != 0)
        ret += kp * error;

    if (ki != 0)
        ret += ki * integral;

    if (kd != 0 && dt > 0)
        ret += kd * errorDerivative;

    ret = (ret < min) ? min : ret;
    ret = (ret > max) ? max : ret;

    return ret;
}