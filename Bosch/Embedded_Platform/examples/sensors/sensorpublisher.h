#ifndef SENSOR_PUBLISHER_H  
#define SENSOR_PUBLISHER_H

#include <DriverTask.h>
#include <mbed.h>
#include <array>
#include <string>
#define PROXIMITYPUBLISHER_NR 5

namespace sensors{
    class ProximityPublisher:public CTask{
        public:
            using SensorArrayT = std::array<CDriverTask<float>*,PROXIMITYPUBLISHER_NR>;
            ProximityPublisher(uint32_t,SensorArrayT,Serial&);

            static void staticSerialCallback(void* obj,char const * a, char * b);
        
        protected:
            void _run();
            void serialCallback(char const * a, char * b);
        private:
            SensorArrayT            m_sensors;
            Serial&                 m_serial;
            bool                    m_isActivate;
    };
};


#endif