#include "DistanceDriver.h"
/* //scalare 1
float a=733.06912872;
float b=-2221.23062859;
float c=2502.20535578;
float d=-1273.63067295;
float e=269.64209866;
*/
//scalare 2 mai precis


DistanceDriver::DistanceDriver(PinName _pin):
pin(_pin)
{
};

DistanceDriver::~DistanceDriver()
{
};

float DistanceDriver::ReadDistance()
{
    float current_value=pin.read();
    //-scalare
    // float scaled_value =   a*current_value*current_value*current_value*current_value*current_value *current_value
    //                      + b*current_value*current_value*current_value*current_value *current_value
    //                      + c*current_value*current_value*current_value *current_value
    //                      + d*current_value*current_value*current_value 
    //                      + e*current_value *current_value
    //                      + f*current_value
    //                      + g ;

    //float scaled_value=current_value; // fara scalare, semnal brut 
    return current_value*5.0;
}