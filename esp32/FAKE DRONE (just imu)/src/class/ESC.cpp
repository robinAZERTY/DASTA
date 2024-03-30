#include "ESC.hpp"

int ESC::nextPwmChannel = 0;

ESC::~ESC()
{
    detach();
}

void ESC::attach(int pin, int min_us, int max_us, int n_bits, int freq, float alpha_work)
{
    // comput the a and b values to convert speed to pwm value (pwm = a*speed + b)
    double max_pwm = pow(2, n_bits) - 1;
    const int min_us_work = min_us + alpha_work * (max_us - min_us);
    a = (max_us - min_us_work) * max_pwm * freq / 1000000.0f;
    b = min_us_work * max_pwm * freq / 1000000.0f;
    min_pwm = min_us * max_pwm * freq / 1000000.0f;

    // setup the pwm channel
    channel = nextPwmChannel++;
    ledcSetup(channel, freq, n_bits);
    ledcAttachPin(pin, channel);
    write(0.0f);
    state = MIN_LIMIT;
    start_up_time = millis();
}

void ESC::detach()
{
    // detach the pwm channel
    ledcDetachPin(channel);
    channel = -1;
    state = DISENGAGED;
    a = 0;
    b = 0;
}

void ESC::write(float speed_command)
{
    this->speed_command = speed_command;
    if (speed_command <= 0)
        ledcWrite(channel, min_pwm); // stop
    else if (speed_command >= 1)
        ledcWrite(channel, a + b); // maximum speed
    else
        ledcWrite(channel, a * speed_command + b); // normal working condition
}

const bool ESC::runArm(const unsigned long time, const unsigned long timeout)
{
    if (time - start_up_time < timeout)
        return false;
    state = ENGAGED;
    return true;
}

void ESC::arm(const unsigned long timeout)
{
    while(!runArm(millis(), timeout))
        delay(10);
};