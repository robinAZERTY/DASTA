#ifndef LOW_PASS_FILTER_HPP
#define LOW_PASS_FILTER_HPP
#include <Arduino.h>
class LowPassFilter
{
public:
    float timeConstant, filteredValue=0;
    bool firstRun = true;

public:
    LowPassFilter(){};
    LowPassFilter(float timeConstant) { setTimeConstant(timeConstant); }
    ~LowPassFilter(){};

    void setTimeConstant(float timeConstant) { this->timeConstant = timeConstant; }
    float getFilteredValue() { 
        if (isinff(filteredValue) || isnanf(filteredValue))
            filteredValue = 0;
        return filteredValue; }
    void reset() { firstRun = true; }
    void filter(float value, float dt)
    {
        if (firstRun)
        {
            filteredValue = value;
            firstRun = false;
            return;
        }
        float alpha = timeConstant / (timeConstant + dt);
        filteredValue = (1 - alpha) * value + alpha * filteredValue;
    }
};

#endif // LOW_PASS_FILTER_HPP