#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <LSM303D.h>
#include <TaskManager.h>
#include <math.h>
// #include <hedl_driver.h>
#include <Encoder.h>
#include <VNH.h>
#include <SerialMonitor.h>
#include <EncoderFilter.h>
#include <Controller.h>
#include <systemmodels.h>

namespace examples{
    class CDISTFTest:public CTask{
        public:
        CDISTFTest( uint32_t        f_period
                    ,Serial&         f_serial)
                    :CTask(f_period)
                    ,m_serial(f_serial)
                    ,m_tf()
                    {
                        std::array<std::array<float,1>,3> l_num({std::array<float,1>({3}),std::array<float,1>({-5.897}),std::array<float,1>({2.9})});
                        std::array<std::array<float,1>,3> l_den({std::array<float,1>({1}),std::array<float,1>({-1.949}),std::array<float,1>({0.9512})});
                        m_tf.setNum(l_num);
                        m_tf.setDen(l_den);
                    }
            
        private:
            void _run(){
                float l_input=1.0;
                float l_output=m_tf(l_input);
                m_serial.printf("%.4f\n",l_output);
            }

            Serial&                                                       m_serial;
            systemmodels::lti::siso::CDiscreteTransferFucntion<float,3,3> m_tf;
            
    };

    class CEncoderSender:public CTask{
        public:
        CEncoderSender(  uint32_t           f_period
                        ,CEncoderFilter&    f_encoder
                        ,task::CControl& f_control
                        ,Serial&            f_serial)
                        :CTask(f_period)
                        ,m_isActived(false)
                        ,m_encoder(f_encoder)
                        ,m_control(f_control)
                        ,m_serial(f_serial)
                        {
                        }
            static void staticSerialCallback(void* obj,char const * a, char * b){
                CEncoderSender* self = static_cast<CEncoderSender*>(obj);
                self->serialCallback(a,b);
            }
        private:
            void serialCallback(char const * a, char * b){
                int l_isActivate=0;
                uint32_t l_res = sscanf(a,"%d",&l_isActivate);
                if(l_res==1){
                    m_isActived=(l_isActivate>=1);
                    sprintf(b,"ack;;");
                }else{
                    sprintf(b,"sintax error;;");
                }
            }
            void _run(){
                if(!m_isActived) return;
                float l_filtRps=m_encoder.getRpsFiltered();
                float l_u=m_control.get();
                // m_serial.printf("@ENPB:%.2f;%.2f;;\r\n",l_filtRps,l_u);  
                m_serial.printf("@ENPB:%.2f;;\r\n",l_filtRps);  
            } 
            bool                m_isActived;
            CEncoderFilter&     m_encoder;
            task::CControl&     m_control;
            Serial&             m_serial;
    };

    class CTask_LSM303: public CTask {
    public:
        CTask_LSM303(   uint32_t        f_period
                        ,LSM303D&       f_lsm303d
                        ,Serial&        f_serial)
                        :CTask(f_period)
                        ,m_lsm303d(f_lsm303d)
                        ,m_serial(f_serial)
                        {
                        }
    private:
        void _run(){
            float l_x,l_y,l_z;
            if(m_lsm303d.read_acce(&l_x,&l_y,&l_z)){
                float sum=sqrt(l_x*l_x+l_y*l_y+l_z*l_z)/1.e3;
                m_serial.printf("@ACDT:%.2f;%.2f;%.2f;%.2f;;\r\n",l_x/1.e3,l_y/1.e3,l_z/1.e3,sum);
            }
            else{
                m_serial.printf("Lsm303D: read problems! Please restart the sensor!\n");
            }
            


        }

        LSM303D&         m_lsm303d;
        Serial&          m_serial;
    };

    class CTask_L3GD20H: public CTask {
    public:
        CTask_L3GD20H(   uint32_t        f_period
                        ,L3GD20H&       f_l3gd20h
                        ,Serial&        f_serial)
                        :CTask(f_period)
                        ,m_l3gd20h(f_l3gd20h)
                        ,m_serial(f_serial)
                        {
                            
                        }
    private:
        void _run(){
            float l_x,l_y,l_z;
            if(m_l3gd20h.readDPS(&l_x,&l_y,&l_z)){
                // float sum=sqrt(l_x*l_x+l_y*l_y+l_z*l_z)/1.e3;
                m_serial.printf("%.2f;%.2f;%.2f;\n",l_x/1.e3,l_y/1.e3,l_z/1.e3);
            }
            else{
                m_serial.printf("wrong settings\n");
            }
            


        }

        L3GD20H&         m_l3gd20h;
        Serial&          m_serial;
    };
};


#endif