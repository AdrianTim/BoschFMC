#include "mbed.h"
#include "SERVO.h"

SERVO::SERVO(PinName _pwm)
:pwm(_pwm)
{
    pwm.period_ms(20);    
    // pwm.write(0.075);
    pwm.write(0.07525);
};

SERVO::~SERVO()
{
};

void SERVO::SetAngle(float angle)
{
    pwm.write(conversion(angle));
};

float SERVO::conversion(float angle)
{
    //rc-servo-4519
    //return (0.0010321101 * angle + 0.0725);
    //fs-5106B
    // return (0.035 +0.0272+ (angle + 25.f) / 1953.135);
    //RS2 MG/BB
    return (0.0009505 * angle + 0.07525);
};

