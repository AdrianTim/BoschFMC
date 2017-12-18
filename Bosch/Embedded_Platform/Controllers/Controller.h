#ifndef CONTROLLER_H
#define CONTROLLER_H

#include<PidController.h>
#include<TaskManager.h>
#include <EncoderFilter.h>
namespace task
{
    class CControl{
        template <class T>
        using  ControllerType=controller::siso::CController<T>;

        public:
            CControl(   CEncoderFilter&                 f_encoder
                            ,ControllerType<double>&         f_pid)
                            :m_encoder(f_encoder)
                            ,m_pid(f_pid)
                            {}
            void setRef(    double                           f_RefRps){
                m_RefRps=f_RefRps;
            }
            double getRef(){
                return m_RefRps;
            }
            double get(){
                return m_u;
            }
            double getError(){
               return m_error;
            }
            void clear(){
                m_pid.clear();
            }
            double    control(){
                double l_MesRps=m_encoder.getRpsFiltered();
                float  l_ref=std::abs(m_RefRps);
                double l_error=l_ref-l_MesRps;
                m_u=convertor(m_pid(l_error));
                m_error=l_error;
                if(m_RefRps<0){
                    m_u=m_u*-1.0;
                }
                return m_u;
            }
        private:
            double convertor(double f_u){
                double l_pwm=f_u;
                // (f_u-(-9.73795803))/2.14326143;
                //Modified  need
                if(l_pwm<-10){
                    return -10;
                }else if(l_pwm>20.0){
                    return 20.0;
                }
                return l_pwm;
            }
            

            CEncoderFilter&             m_encoder;
            ControllerType<double>&     m_pid;
            double                       m_RefRps;
            double                       m_u;
            double                       m_error;

    };
}


#endif