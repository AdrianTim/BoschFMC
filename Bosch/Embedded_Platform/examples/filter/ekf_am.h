#ifndef EKF_AM_H
#define EKF_AM_H

#include<filter.h>
#include<ackermannmodel.h>

namespace examples{
    namespace filter{
        template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
        using CEKF  =   ::filter::ltv::mimo::CEKF<T,NA,NB,NC>;
        // Extended Kalman Filter aplied on the Ackermann Model
        class CEKF_AM:public CEKF<double,2,10,5>{
            private:
                using CAckermannModel = ::examples::systemmodels::ackermannmodel::CAckermannModel;
                using CJMAckermannModel = ::examples::systemmodels::ackermannmodel::CJMAckermannModel;
            public:
                CEKF_AM(    CAckermannModel&            f_ackermannModel
                            ,CJMAckermannModel          f_jacobianMatrixCalc
                            ,const CJMTransitionType&                   f_Q
                            ,const CObservationNoiseType&               f_R)
                    :CEKF<double,2,10,5>(f_ackermannModel,f_jacobianMatrixCalc,f_Q,f_R)
                    {}
        };
    };
};


#endif