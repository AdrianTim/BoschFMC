#include "SplineInterpreter.h"

void CSplineInterpreter::setData(   bool        f_isForward,
                                    float       f_a_x,
                                    float       f_a_y,
                                    float       f_b_x,
                                    float       f_b_y,
                                    float       f_c_x,
                                    float       f_c_y,
                                    float       f_d_x,
                                    float       f_d_y,
                                    float       f_duration_sec){
    this->data.isForward=f_isForward;
    this->data.a=std::complex<float>(f_a_x,f_a_y);
    this->data.b=std::complex<float>(f_b_x,f_b_y);
    this->data.c=std::complex<float>(f_c_x,f_c_y);
    this->data.d=std::complex<float>(f_d_x,f_d_y);
    this->data.duration_sec=f_duration_sec;
    this->isNewData=true;
}


CSplineInterpreter::SplineInterpreter_Data CSplineInterpreter::getData(){
    this->isNewData=false;
    return this->data;
}


void CSplineInterpreter::serialCallback(char const * f_message, char * f_response){
     float a_x,a_y,b_x,b_y,c_x,c_y,d_x,d_y,duration_sec;
     int isForward=1;
     int32_t nrData=sscanf(f_message,"%d;%f;%f;%f;%f;%f;%f;%f;%f;%f",
                                    &isForward,
                                    &a_x,
                                    &a_y,
                                    &b_x,
                                    &b_y,
                                    &c_x,
                                    &c_y,
                                    &d_x,
                                    &d_y,
                                    &duration_sec);
    if(10==nrData && duration_sec>0 && (isForward==1 || isForward==0)){
        setData(isForward,a_x,a_y,b_x,b_y,c_x,c_y,d_x,d_y,duration_sec);
        sprintf(f_response,"ack;;");
    }
    else{
        sprintf(f_response,"sintax error;;");
    }
}