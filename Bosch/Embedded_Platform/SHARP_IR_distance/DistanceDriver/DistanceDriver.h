#ifndef DRIVER_H
#define DRIVER_H

#include <mbed.h>

class DistanceDriver
{
public:
    DistanceDriver(PinName pin);
    ~DistanceDriver();
    float ReadDistance(void);

private:
    AnalogIn pin;
    const float a=28953515.3962151;
    const float b=-74877355.57409884;
    const float c=79154205.11187918;
    const float d=-43850966.0842832;
    const float e=13447642.79808201;
    const float f=-2167605.28462039;
    const float g=143704.47476248;


};

#endif