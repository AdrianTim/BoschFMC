#ifndef _VNH_
#define _VNH_
#include "mbed.h"
#include <cmath>

class VNH
{
public:
    VNH(PinName, PinName, PinName, PinName);
    ~VNH();
    void Start(void);
    void Stop(void);
    void Run(float speed);
    void Brake();
    void Inverse(float f_speed);
    float GetCurrent(void);
    
private:
    void Go(float speed);
    float increment;
    float current_speed;
    PwmOut      pwm;
    DigitalOut  ina;
    DigitalOut  inb;
    AnalogIn    current;
};

#endif