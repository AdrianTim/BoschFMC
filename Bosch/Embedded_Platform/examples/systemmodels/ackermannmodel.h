#ifndef ACKERMANN_MODEL_H
#define ACKERMANN_MODEL_H

#include <math.h>
#include<systemmodels.h>
#include"ackermanntypes.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#ifndef DEG2RAD 
    #define DEG2RAD M_PI/180.0
#endif


namespace examples{
    namespace systemmodels{
        namespace ackermannmodel{
            template<class T,uint32_t NA,uint32_t NB,uint32_t NC>
            using CSystemModelType              =   ::systemmodels::nlti::mimo::CDiscreteTimeSystemModel<T,NA,NB,NC>;
            template<class T,uint32_t NA,uint32_t NB,uint32_t NC>
            using CJacobianMatricesType         =   ::systemmodels::nlti::mimo::CJacobianMatrices<T,NA,NB,NC>;

            class CAckermannModel:public CSystemModelType<double,2,10,5>{
                public:
                    CAckermannModel(const double       f_dt     
                                    ,const double      f_gamma
                                    ,const double      f_wb
                                    ,const double      f_b
                                    ,const double      f_J
                                    ,const double      f_K
                                    ,const double      f_R
                                    ,const double      f_L)
                                    :CSystemModelType<double,2,10,5>(f_dt)
                                    ,m_gamma(f_gamma)
                                    ,m_wb(f_wb)
                                    ,m_bJ(f_b/f_J)
                                    ,m_KJ(f_K/f_J)
                                    ,m_KL(f_K/f_L)
                                    ,m_RL(f_R/f_L)
                                    ,m_L(f_L){}
                    CAckermannModel(const CStatesType&      f_states
                                    ,const double            f_dt     
                                    ,const double            f_gamma
                                    ,const double            f_wb
                                    ,const double            f_b
                                    ,const double            f_J
                                    ,const double            f_K
                                    ,const double            f_R
                                    ,const double            f_L)
                                    :CSystemModelType<double,2,10,5>(f_states,f_dt)
                                    ,m_gamma(f_gamma)
                                    ,m_wb(f_wb)
                                    ,m_bJ(f_b/f_J)
                                    ,m_KJ(f_K/f_J)
                                    ,m_KL(f_K/f_L)
                                    ,m_RL(f_R/f_L)
                                    ,m_L(f_L){}
                    
                    
                    CStatesType        update(const CInputType&       f_input){
                        CState l_states=m_states;
                        CInput l_input=f_input;
                        l_states.x()+=m_dt*l_states.x_dot();
                        l_states.y()+=m_dt*l_states.y_dot();

                        l_states.x_dot_prev()=l_states.x_dot();
                        l_states.y_dot_prev()=l_states.y_dot();

                        l_states.x_dot()=m_gamma*l_states.omega()*cos(l_states.teta_rad());
                        l_states.y_dot()=m_gamma*l_states.omega()*sin(l_states.teta_rad());

                        double l_alpha_rad=l_input.alpha()*DEG2RAD;
                        l_states.teta_rad_dot()=m_gamma*l_states.omega()*tan(l_alpha_rad)/m_wb;
                        l_states.teta_rad()+=m_dt*l_states.teta_rad_dot();

                        double omega_k_1=(1-m_dt*m_bJ)*l_states.omega()+m_dt*m_KJ*l_states.i();//next state of the motor's rotation speed
                        l_states.i()=-1*m_dt*m_KL*l_states.omega()+(1-m_dt*m_RL)*l_states.i()+m_dt*l_input.v()/m_L;
                        l_states.omega()=omega_k_1;
                        m_states=l_states;
                        return l_states;
                    }



                    COutputType        calculateOutput(const CInputType&  f_input){
                        CState l_states=m_states;
                        // CInput l_input=f_input;
                        COutput l_outputs(COutputType::zeros());
                        l_outputs.x_ddot()=l_states.x_dot()-l_states.x_dot_prev();
                        l_outputs.y_ddot()=l_states.y_dot()-l_states.y_dot_prev();
                        l_outputs.teta_rad_dot()=l_states.teta_rad_dot();
                        l_outputs.speed()=l_states.omega()*m_gamma;
                        // output.alpha()=f_input.alpha();
                        l_outputs.i()=l_states.i();
                        m_outputs=l_outputs;
                        return l_outputs;
                    }
                private:
                    // gamma=Meter/Rotation
                    const double m_gamma;
                    // Wheel base distance in meter
                    const double m_wb;
                    // Motor Constants
                    const double m_bJ,m_KJ,m_KL,m_RL,m_L;      
            };

            class CJMAckermannModel:public CJacobianMatricesType<double,2,10,5>{
                public:
                    CJMAckermannModel   (double          f_dt
                                        ,double          f_gamma
                                        ,double          f_wb
                                        ,double          f_b
                                        ,double          f_J
                                        ,double          f_K
                                        ,double          f_R
                                        ,double          f_L)
                                        :m_dt(f_dt)
                                        ,m_gamma(f_gamma)
                                        ,m_wb(f_wb)
                                        ,m_bJ(f_b/f_J)
                                        ,m_KJ(f_K/f_J)
                                        ,m_KL(f_K/f_L)
                                        ,m_RL(f_R/f_L)
                    {
                        m_ObservationMatrix=initObservationMatrix();
                    }
                


                    CJMObservationType getJMObservation(    const CStatesType&       f_states
                                                            ,const CInputType&       f_input){
                        return m_ObservationMatrix;
                    }
                
                private:
                    linalg::CMatrix<double,5,10> initObservationMatrix(){
                        linalg::CMatrix<double,5,10> l_data;
                        l_data[0][2]=l_data[1][3]=1.f/m_dt;
                        l_data[0][4]=l_data[1][5]=-1.f/m_dt;
                        l_data[2][7]=1.f;
                        l_data[3][8]=m_gamma;
                        l_data[4][9]=1.f;
                        return l_data;
                    }

                    inline void  setJacobianStateMatrixWithOne(CJMTransitionType& f_matrix){
                        f_matrix[0][0]=f_matrix[1][1]=1;
                        f_matrix[4][2]=f_matrix[5][3]=1;
                        f_matrix[6][6]=1;
                        return;
                    }

                public:
                    CJMTransitionType getJMTransition(       const CStatesType&       f_states
                                                            ,const CInputType&       f_input){

                        CJMTransitionType l_data(CJMTransitionType::zeros());
                        setJacobianStateMatrixWithOne(l_data);
                        CState l_states(f_states);
                        CInput l_input(f_input);

                        //Setting values in the jacobian matrix
                        l_data[0][2]=m_dt;
                        l_data[1][3]=m_dt;
                        
                        l_data[2][6]=-m_gamma*l_states.omega()*sin(l_states.teta_rad());
                        l_data[2][8]=m_gamma*cos(l_states.teta_rad());

                        l_data[3][6]=m_gamma*l_states.omega()*cos(l_states.teta_rad());
                        l_data[3][8]=m_gamma*sin(l_states.teta_rad());

                        double l_alpha_rad=l_input.alpha()*DEG2RAD;
                        l_data[7][8]=l_data[6][8]=m_dt*m_gamma*tan(l_alpha_rad)/m_wb;
                        //l_data[7][11]=l_data[6][11]=m_dt*m_gamma*f_state.omega/(m_wb*pow(cos(l_alpha_rad),2));

                        l_data[8][8]=(1-m_dt*m_bJ);
                        l_data[8][9]=m_dt*m_KJ;
                        l_data[9][8]=m_dt*m_KL;
                        l_data[9][9]=(1-m_dt*m_RL);
                        //l_data[9][10]=m_dt;
                        return l_data;
                    }

                private:
                    //Constant values
                    // Time step
                    const double m_dt;
                    // gamma=Meter/Rotation
                    const double m_gamma;
                    // Wheel base distance in meter
                    const double m_wb;
                    // Motor Constants
                    const double m_bJ,m_KJ,m_KL,m_RL;
                    //Please leave this matrix to be the last member in this class, as it will be initialized
                    linalg::CMatrix<double,5,10> m_ObservationMatrix;    
            };
        };
    };
};

#include "ackermannmodel.inl"
#endif