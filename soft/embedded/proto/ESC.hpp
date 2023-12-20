class ESC
{
private:
    int pin = -1;
    float u = 0;

public:
    ESC(const int pin);
    ESC();
    ~ESC();
    const bool getU() {return u; };
    void attach(const int pin);
    void set(const float power);
};