#include <Arduino.h>


/*
blHeli S config :
- min: 1080
- max: 2020
*/
class ESC
{
public:
    ESC() = default;
    ESC(int pin, int min_us = 1000, int max_us = 2020, int n_bits = 16, int freq = 50, float alpha_work = 0.083) { attach(pin, min_us, max_us, n_bits, freq, alpha_work); };
    ~ESC();

    /*
     * @brief: attach the ESC to a pin
     * @param pin: pin to which the ESC is attached
     * @param min_ms: minimum pulse width in milliseconds, default is 1000
     * @param max_ms: maximum pulse width in milliseconds, default is 2000
     * @param n_bits: resolution of the pwm signal, default is 16
     * @param freq: frequency of the pwm signal, default is 50
     * @param alpha_work: percentage of the pulse width on witch the motor begin to work well, default is 0.065 (6.5%)
     * @param freq: frequency of the pwm signal, default is 50
     */
    void attach(int pin, int min_us = 1000, int max_us = 2020, int n_bits = 16, int freq = 50, float alpha_work = 0.083);

    /*
     * @brief: arm the ESC
     * @param time: time in milliseconds, default is the current time measured by millis()
     */
    const bool runArm(const unsigned long time = millis(), const unsigned long timeout = 1000);

    // blocking version of runArm
    void arm(const unsigned long timeout = 3000);

    /*
     * set the speed of the ESC
     * @param speed: speed of the ESC, between 0 and 1
     */
    void write(float speed_command);

    /*
     * @brief: get the speed of the ESC, between 0 and 1 (for now, the speed is the last speed set by write())
     * @return: the speed of the ESC
     */
    const float read() const { return speed_command; };

    /*
     * @brief: check if the ESC is attached to a pin
     * @return: true if the ESC is attached to a pin, false otherwise
     */
    const bool attached() const { return channel != -1; };

    /*
     * @brief: check if the ESC is armed
     * @return: true if the ESC is armed, false otherwise
     */
    const bool engaged() const { return state == ENGAGED; };

private:
    static int nextPwmChannel;         // unique pwm channels for each instance
    static const float min_th, max_th; // min and max pulse width in seconds
    unsigned long start_up_time;       // time at which the ESC started the arming process

    // initialization steps
    enum State
    {
        DISENGAGED = 0,
        MIN_LIMIT,
        MAX_LIMIT,
        ENGAGED
    };
    State state = DISENGAGED;   // DISENGAGED; // state of the ESC
    int channel = -1;        // pwm channel of the ESC
    float a = 0, b = 0;      // parameters to convert speed to pwm value (f(x) = ax + b)
    float speed_command = 0; // speed of the ESC
    uint32_t min_pwm = 0;    // minimum pwm value for the ESC to work
    // detach the ESC from the pin, private because it can be dangerous to detach the ESC while it is armed
    void detach();
};