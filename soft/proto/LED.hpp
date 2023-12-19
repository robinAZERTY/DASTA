class LED
{
private:
    int pin = -1;
    float u = 0;//current command

public:
    LED(const int pin);
    LED();
    ~LED();
    const bool getU() {return u; };
    void attach(const int pin);
    void set(const float power);
};