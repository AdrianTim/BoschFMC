#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <linalg.h>
#include <systemmodels.h>

namespace controller{
    namespace siso{
        template<class T>
        class CController{
            public:
                virtual T operator()(const T&)=0;
                virtual void clear()=0;
        };

       
        template<class T>
        class CPidController:public CController<T>{
            public:
                using CIntegratorTF     =systemmodels::lti::siso::CDiscreteTransferFucntion<T,2,2>;
                using CDerivatorTF      =systemmodels::lti::siso::CDiscreteTransferFucntion<T,2,2>;
                CPidController(          T               f_kp
                                        ,T              f_ki
                                        ,T              f_kd
                                        ,T              f_tf
                                        ,T              f_dt)
                                        :m_kp(f_kp)
                                        ,m_intTf()
                                        ,m_derTf()
                                        ,m_dt(f_dt)
                {
                    linalg::CMatrix<T,2,1> l_numIntM({std::array<T,1>({f_ki*f_dt}),std::array<T,1>({f_ki*f_dt})});
                    m_intTf.setNum(l_numIntM);
                    linalg::CMatrix<T,2,1> l_denIntM({std::array<T,1>({2}),std::array<T,1>({-2})});
                    m_intTf.setDen(l_denIntM);

                    linalg::CMatrix<T,2,1> l_numDerM({std::array<T,1>({2*f_kd}),std::array<T,1>({-2*f_kd})});
                    m_derTf.setNum(l_numDerM);
                    linalg::CMatrix<T,2,1> l_denDerM({std::array<T,1>({2*f_tf+f_dt}),std::array<T,1>({f_dt-2*f_tf})});
                    m_derTf.setDen(l_denDerM);
                    
                    // linalg::CMatrix<T,2,1>({std::array<T,1>({2*f_kd}),std::array<T,1>({-2*f_kd})}),linalg::CMatrix<T,2,1>({std::array<T,1>({f_tf*2+f_dt}),std::array<T,1>({f_dt-2*f_tf})})
                }

                T operator()(const T& f_input){
                    return m_kp*f_input+m_intTf(f_input)+m_derTf(f_input);
                }
            static void staticSerialCallback(void* obj,char const * a, char * b)
            {
                CPidController* self = static_cast<CPidController*>(obj);
                self->serialCallback(a,b);
            }

            void clear(){
                m_intTf.clearMemmory();
                m_derTf.clearMemmory();
            }

            private:

            void setController(         T               f_kp
                                        ,T              f_ki
                                        ,T              f_kd
                                        ,T              f_tf)
            {
                m_kp=f_kp;
                linalg::CMatrix<T,2,1> l_numIntM({std::array<T,1>({f_ki*m_dt}),std::array<T,1>({f_ki*m_dt})});
                m_intTf.setNum(l_numIntM);
                linalg::CMatrix<T,2,1> l_denIntM({std::array<T,1>({2}),std::array<T,1>({-2})});
                m_intTf.setDen(l_denIntM);

                linalg::CMatrix<T,2,1> l_numDerM({std::array<T,1>({2*f_kd}),std::array<T,1>({-2*f_kd})});
                m_derTf.setNum(l_numDerM);
                linalg::CMatrix<T,2,1> l_denDerM({std::array<T,1>({2*f_tf+m_dt}),std::array<T,1>({m_dt-2*f_tf})});
                m_derTf.setDen(l_denDerM);
            }

            void serialCallback(char const * a, char * b)
            {
                float l_kp,l_ki,l_kd,l_tf;
                uint32_t l_res = sscanf(a,"%f;%f;%f;%f;",&l_kp,&l_ki,&l_kd,&l_tf);
                if (4 == l_res)
                {
                    setController(l_kp,l_ki,l_kd,l_tf);
                    sprintf(b,"ack;;%2.5f;%2.5f;%2.5f;%2.5f;",l_kp,l_ki,l_kd,l_tf);
                }
                else
                {
                    sprintf(b,"sintax error;;");
                }
            }
                T                       m_kp;
                CIntegratorTF           m_intTf;
                CDerivatorTF            m_derTf;
                float                   m_dt;

        };
    }
}


#endif