class PID
{
public:
    PID();
    ~PID();
    float kp = 0;                // Proportional gain (no unit)
    float ki = 0;                // Integral gain (1/s)
    float kd = 0;                // Derivative gain (s)
    float lpf_time_constant = 0; // Low pass filter time constant (s) used to filter the derivative term
    float integral_min = 0;      // Integral minimum value to avoid windup
    float integral_max = 0;      // Integral maximum value to avoid windup
    float output_min = 0;        // Output minimum value to avoid windup
    float output_max = 0;        // Output maximum value to avoid windup

    void setIntegralLimit(float min, float max);
    void setOutputLimit(float min, float max);
    float compute(float ref, float val, float ref_dot, float val_dot, float dt);
    float compute(float ref, float val, float dt);
    float compute(float error, float dt);
    void reset();

private:
    float integral = 0;
    float prev_error = 0;
    float prev_val = 0;
};