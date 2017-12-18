#include "sensorpublisher.h"

using namespace sensors;
ProximityPublisher::ProximityPublisher( uint32_t                                                 f_period
                                        ,SensorArrayT                                            f_sensors
                                        ,Serial&                                                 f_serial)
                                        :CTask(f_period)
                                        ,m_sensors(f_sensors)
                                        ,m_serial(f_serial)
                                        ,m_isActivate(false)
{
}

void ProximityPublisher::staticSerialCallback(void* obj,char const * a, char * b)
{
    ProximityPublisher* self = static_cast<ProximityPublisher*>(obj);
    self->serialCallback(a,b);
}


void ProximityPublisher::serialCallback(char const * a, char * b){
    int l_isActivate=0;
    uint32_t l_res = sscanf(a,"%d",&l_isActivate);
    if(l_res==1){
        m_isActivate=(l_isActivate>=1);
        sprintf(b,"ack;;");
    }else{
        sprintf(b,"sintax error;;");
    }
}


void ProximityPublisher::_run(){
    if(!m_isActivate) return;
    char l_buf[100];
    sprintf(l_buf,"@DSPB:");
    for(uint8_t i=0;i<PROXIMITYPUBLISHER_NR;++i){
        
        sprintf(l_buf,"%s%2.2f;",l_buf,m_sensors[i]->getDistanceMeasured());
    }
    sprintf(l_buf,"%s;\n\r",l_buf);
    m_serial.printf("%s",l_buf);

}