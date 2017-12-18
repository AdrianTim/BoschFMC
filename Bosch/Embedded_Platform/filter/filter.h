#ifndef FILTER_H
#define FILTER_H

#include <linalg.h>
#include <systemmodels.h>

namespace filter
{
    template<class T>
    class CFilterFunction
    {
    public:
        virtual T operator()(T&)=0; 
    }; 
    namespace lti
    {
        namespace siso
        {
           
            template <class T, uint32_t NA, uint32_t NB>
            class CIIRFilter:public CFilterFunction<T>
            {
            public:
                CIIRFilter(const linalg::CRowVector<T,NA>& f_A,const linalg::CRowVector<T,NB>& f_B) : m_A(f_A), m_B(f_B), m_Y(), m_U() {}
                T operator()(T& f_u)
                {
                    for (uint32_t l_idx = NB-1; l_idx > 0; --l_idx)
                    {
                        m_U[l_idx] = m_U[l_idx-1];
                    }
                    m_U[0][0] = f_u;

                    linalg::CMatrix<T,1,1> l_y = m_B*m_U - m_A*m_Y;
                    // T l_y = m_B*m_U - m_A*m_Y;

                    for (uint32_t l_idx = NA-1; l_idx > 0 ; --l_idx)
                    {
                        m_Y[l_idx] = m_Y[l_idx-1];
                    }
                    m_Y[0][0] = l_y[0][0];

                    return m_Y[0][0];
                }
            private:
                CIIRFilter(){}
                linalg::CRowVector<T,NA> m_A;
                linalg::CRowVector<T,NB> m_B;
                linalg::CColVector<T,NA> m_Y;
                linalg::CColVector<T,NB> m_U;
            };
            template <class T, uint32_t NB>
            class CFIRFilter:public CFilterFunction<T>
            {
            public:
                CFIRFilter(const linalg::CRowVector<T,NB>& f_B) : m_B(f_B), m_U() {}
                T operator()(T& f_u)
                {
                    for (uint32_t l_idx = NB-1; l_idx > 0; --l_idx)
                    {
                        m_U[l_idx] = m_U[l_idx-1];
                    }
                    m_U[0] = f_u;

                    linalg::CMatrix<T,1,1> l_y = m_B*m_U;

                    return l_y[0][0];
                }
            private:
                CFIRFilter() {}
                linalg::CRowVector<T,NB> m_B;
                linalg::CColVector<T,NB> m_U;
            };
            template <class T, uint32_t NB>
            class CMeanFilter:public CFilterFunction<T>
            {
            public:
                CMeanFilter() : m_B(1./NB), m_U() {}

                virtual T operator()(T& f_u)
                {
                    T l_y =0;

                    for (uint32_t l_idx = 1; l_idx < NB; ++l_idx)
                    {
                        m_U[l_idx][0] = m_U[l_idx-1][0];
                        l_y += m_U[l_idx][0];
                    }
                    m_U[0][0] = f_u;
                    l_y += m_U[0][0];

                    return m_B*l_y;
                }
            private:
                T m_B;
                linalg::CColVector<T,NB> m_U;
            };
            /*template <class T, uint32_t NB>
            class CMedianFilter
            {

            };*/
        };
        namespace mimo
        {
            template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
            class CSSModel
            {
            public:

                using CStateType = linalg::CColVector<T,NA>;
                using CStateTransitionType = linalg::CMatrix<T,NA,NA>;
                using CInputType = linalg::CColVector<T,NB>;
                using CMeasurementType = linalg::CColVector<T,NC>;
                using CInputMatrixType = linalg::CMatrix<T,NA,NB>;
                using CMeasurementMatrixType = linalg::CMatrix<T,NC,NA>;
                using CDirectTransferMatrixType = linalg::CMatrix<T,NC,NB>;

                CSSModel(
                    const CStateTransitionType& f_stateTransitionMatrix,
                    const CInputMatrixType& f_inputMatrix,
                    const CMeasurementMatrixType& f_measurementMatrix
                ) 
                : m_stateVector()
                , m_stateTransitionMatrix(f_stateTransitionMatrix)
                , m_inputMatrix(f_inputMatrix)
                , m_measurementMatrix(f_measurementMatrix)
                , m_directTransferMatrix()
                {
                    // do nothing
                }

                CSSModel(
                    const CStateTransitionType& f_stateTransitionMatrix,
                    const CInputMatrixType& f_inputMatrix,
                    const CMeasurementMatrixType& f_measurementMatrix,
                    const CDirectTransferMatrixType& f_directTransferMatrix
                ) 
                : m_stateVector()
                , m_stateTransitionMatrix(f_stateTransitionMatrix)
                , m_inputMatrix(f_inputMatrix)
                , m_measurementMatrix(f_measurementMatrix)
                , m_directTransferMatrix(f_directTransferMatrix)
                {
                    // do nothing
                }

                CSSModel(
                    const CStateTransitionType& f_stateTransitionMatrix,
                    const CInputMatrixType& f_inputMatrix,
                    const CMeasurementMatrixType& f_measurementMatrix,
                    const CDirectTransferMatrixType& f_directTransferMatrix,
                    const CStateType& f_state                    
                ) 
                : m_stateVector(f_state)
                , m_stateTransitionMatrix(f_stateTransitionMatrix)
                , m_inputMatrix(f_inputMatrix)
                , m_measurementMatrix(f_measurementMatrix)
                , m_directTransferMatrix(f_directTransferMatrix)
                {
                    // do nothing
                }

                const CStateType& state() const {return m_stateVector;} 
                CStateType& state() {return m_stateVector;} 

                CMeasurementType operator()(const CInputType& f_inputVector)
                {
                    updateState(f_inputVector);

                    return getOutput(f_inputVector);
                }

                void updateState(const CInputType& f_inputVector)
                {
                  m_stateVector = m_stateTransitionMatrix * m_stateVector + m_inputMatrix * f_inputVector;
                }

                CMeasurementType getOutput(const CInputType& f_inputVector)
                {
                    return m_measurementMatrix * m_stateVector + m_directTransferMatrix * f_inputVector;
                }

            private:
                CSSModel() {}
                CStateType m_stateVector;
                CStateTransitionType m_stateTransitionMatrix;
                CInputMatrixType m_inputMatrix;
                CMeasurementMatrixType m_measurementMatrix;
                CDirectTransferMatrixType m_directTransferMatrix;
            };

            template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
            class CKalmanFilter
            {
            public:
              using CStateType                 = linalg::CVector<T, NA>;
              using CInputVectorType           = linalg::CVector<T, NB>;
              using COutputVectorType          = linalg::CVector<T, NC>;
              using CInputType                 = linalg::CMatrix<T, NA, NB>;
              using CControInputType           = linalg::CMatrix<T, NA, NC>;
              using CModelCovarianceType       = linalg::CMatrix<T, NA, NA>;
              using CMeasurementCovarianceType = linalg::CMatrix<T, NC, NC>;
              using CStateTransType            = linalg::CMatrix<T, NA, NA>;
              using CMeasurementType           = linalg::CMatrix<T, NC, NA>;
              using CKalmanGainType            = linalg::CMatrix<T, NA, NC>;

              CKalmanFilter(
                  const CStateTransType&            f_stateTransitionModel,
                  const CControInputType&           f_controlInput,
                  const CMeasurementType&           f_observationModel,
                  const CModelCovarianceType&       f_covarianceProcessNoise,        // <-
                  const CMeasurementCovarianceType& f_observationNoiseCovariance     // <-

              )
              : m_stateTransitionModel(f_stateTransitionModel)
              , m_controlInput(f_controlInput)
              , m_observationModel(f_observationModel)
              , m_covarianceProcessNoise(f_covarianceProcessNoise)
              , m_observationNoiseCovariance(f_observationNoiseCovariance)
              {
              }

              const CStateType& state() const
              {
                  return m_posterioriState;
              }

              CStateType& state()
              {
                  return m_posterioriState;
              }

              CMeasurementType operator()(const CInputVectorType& f_input)
              {
                  predict(f_input);
                  return update();
              }

              CMeasurementType operator()()
              {
                  predict();
                  return update();
              }

              void predict()
              {
                  m_previousState      = m_prioriState;
                  m_previousCovariance = m_prioriCovariance;
                  m_prioriState        = m_stateTransitionModel * m_previousState;
                  m_prioriCovariance   = m_stateTransitionModel * m_previousCovariance * transpose(m_stateTransitionModel) + m_covarianceProcessNoise;
              }

              void predict(const CInputVectorType& f_input)
              {
                  m_previousState      = m_prioriState;
                  m_previousCovariance = m_prioriCovariance;
                  m_prioriState        = m_stateTransitionModel * m_previousState + m_controlInput * f_input;
                  m_prioriCovariance   = m_stateTransitionModel * m_previousCovariance * transpose(m_stateTransitionModel) + m_covarianceProcessNoise;
              }

              const CMeasurementType& update(void)
              {
                  m_measurementResidual  = m_measurement - m_observationModel * m_prioriState;
                  m_measurement          = m_observationModel * m_posterioriState;
                  m_residualCovariance   = m_observationModel * m_prioriCovariance * transpose(m_observationModel) + m_observationNoiseCovariance;
                  m_kalmanGain           = m_prioriCovariance * transpose(m_observationModel) * m_residualCovariance.inv();
                  m_posterioriState      = m_prioriState + m_kalmanGain * m_measurementResidual;
                  m_posterioriCovariance = ( CStateTransType::eye() - m_kalmanGain * m_observationModel ) * m_prioriCovariance;
                  return m_measurementResidual;
              }

            private:
              CKalmanFilter() {}

              CStateType                 m_previousState;              // previous state
              CStateType                 m_prioriState;                // priori state
              CStateType                 m_posterioriState;            // posteriori state
              CControInputType           m_controlInput;               // control input modelþ
              CModelCovarianceType       m_previousCovariance;         // previous covariance estimate         // <-
              CModelCovarianceType       m_prioriCovariance;           // priori covariance estimate           // <-
              CModelCovarianceType       m_posterioriCovariance;       // posteriori covariance estimate       // <-
              CStateTransType            m_stateTransitionModel;       // state transition model
              CModelCovarianceType       m_covarianceProcessNoise;     // covariance of process noise          // done
              CMeasurementType           m_measurementResidual;        // innovation or measurement residual
              CMeasurementType           m_measurement;                // observation (or measurement)
              CMeasurementType           m_observationModel;           // observation model
              CMeasurementCovarianceType m_residualCovariance;         // innovation or residual covariance    // <-
              CMeasurementCovarianceType m_observationNoiseCovariance; // covariance of observation noise      // <-
              CKalmanGainType            m_kalmanGain;                 // optimal kalman gain

            };

        };
    };
    namespace ltv
    {
        namespace mimo
        {
            // Extended Kalman Filter
            // Template parameters:
            //              - T     variable type
            //              - NA    nr input
            //              - NB    nr states
            //              - NC    nr output
            // Input parameters
            template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
            class CEKF
            {
                public:
                    using CSystemModelType              =   systemmodels::nlti::mimo::CDiscreteTimeSystemModel<T,NA,NB,NC>;
                    using CJacobianMatricesType         =   systemmodels::nlti::mimo::CJacobianMatrices<T,NA,NB,NC>;
                    using CStatesType                   =   linalg::CMatrix<T,NB,1>;
                    using CInputType                    =   linalg::CMatrix<T,NA,1>;
                    using COutputType                   =   linalg::CMatrix<T,NC,1>;
                    using CJMTransitionType             =   linalg::CMatrix<T,NB,NB>;
                    using CJMObservationType            =   linalg::CMatrix<T,NC,NB>;
                    using CObservationNoiseType         =   linalg::CMatrix<T,NC,NC>;
                    using CKalmanGainType               =   linalg::CMatrix<T,NC,NB>;
                    CEKF(   CSystemModelType&                           f_systemModel
                            ,CJacobianMatricesType&                     f_jbMatrices
                            ,const CJMTransitionType&                   f_Q
                            ,const CObservationNoiseType&               f_R)
                        :m_systemModel(f_systemModel)
                        ,m_jbMatrices(f_jbMatrices)
                        ,m_covarianceMatrix(linalg::CMatrix<T,NB,NB>::ones())
                        ,m_Q(f_Q)
                        ,m_R(f_R)
                        {}
                    
                    
                    void predict    (const CInputType&         f_input);
                    void update     (const CInputType&         f_input
                                    ,const COutputType&        f_measurment);
                protected:
                private:
                    CSystemModelType&                    m_systemModel;
                    CJacobianMatricesType&               m_jbMatrices;
                    //Predicted covariance matrix
                    CJMTransitionType                   m_covarianceMatrix;
                    //The covariance of the process noise
                    CJMTransitionType                   m_Q;
                    //The covariance of the observation noise
                    CObservationNoiseType               m_R;

            };
        };
    };

    namespace nonlinear
    {
        namespace siso
        {         
            template <class T, uint32_t N>
            class CMedianFilter:public CFilterFunction<T>
            {
            public:
                CMedianFilter() :   m_median(), m_smallest(),m_new(0),m_size(N), m_coada()
                {
                    // for (unsigned int l_idx = 0; l_idx < N; l_idx++)
                    // {
                    //     m_coada[l_idx] = new structura;
                        
                    // }

                    for (unsigned int l_idx = 0; l_idx < N ; l_idx++)
                    {
                        m_coada[l_idx].info = 0;
                        m_coada[l_idx].next = &(m_coada[(l_idx + 1) % N]);
                        m_coada[l_idx].prev = &(m_coada[(N + l_idx - 1) % N]);
                    }

                    m_new = N - 1;
                    m_size = N;
                    m_median = &(m_coada[m_size / 2]);
                    m_smallest =&(m_coada[0]);
                }

                inline T addNewValue(T& f_val);
                inline T getMedian();
                inline T operator()(T& f_v);

            private:
                struct structura
                {
                    T info;
                      structura *next;
                      structura *prev;
                    
                };

                  structura *m_median;		  //pointer la elementul din lista care e mediana
                  structura *m_smallest;        //pionter la cel mai mic elemt din lista (primul)
                  uint32_t m_new;		  //index la cel mai nou element din coada
                  uint32_t m_size;		  //marimea cozii/listei

                structura m_coada[N];
                
            };

            /*
            template <class T, uint32_t N>
            T CMedianFilter<T, N>::operator()(T& f_v)
            {
                return this.addNewValue(f_v);
            }
            */
            template <class T, uint32_t N>
            //T CMedianFilter<T, N>::addNewValue(T& f_val)
            T CMedianFilter<T, N>::operator()(T& f_val)
            {
                m_new = (m_new + 1) % m_size;		 //shift new index	 //inca e valoarea veche
            /* // varianta pentru a decide valoarea mediana eficient-----eficient daca filtrul are dimensiuni mari
            //	->V2 start	  restructurare          EXISTA CAUZE CARE NU SUNT TRATATE CORECT SAU NETRATATE COMPLET!!!!!!!!!!!
                if ((m_coada[m_new]->info > m_median->info) && (f_val <= m_median->info))
                {
                    if (f_val > m_median->prev->info)
                    {
                        m_median = m_coada[m_new];		   //med=new
                    }
                    else
                    {
                        m_median = m_median->prev;		   // <-
                    }
                }
                else if ((m_coada[m_new]->info < m_median->info) && (f_val > m_median->info))
                {
                    if (f_val > m_median->next->info)
                    {
                        m_median = m_median->next;		  // ->
                    }
                    else
                    {   
                        m_median = m_coada[m_new];		  //med=new
                    }
                }
                else if ((m_coada[m_new]->info == m_median->info))
                {
                    if ((f_val < m_median->info)&&(f_val <= m_median->prev->info))
                    {
                        m_median = m_median->prev;		   // <-
                    }
                    else if (f_val > m_median->info)
                        {
                            if (f_val <= m_median->next->info)
                            {
                                m_median = m_coada[m_new];		  //med=new
                            }
                            else
                            {
                                m_median = m_median->next;		  // ->
                            }
                        }
                        else
                        {
                            m_median = m_coada[m_new];		  //med=new
                        }
                }
                */
                m_coada[m_new ].info = f_val;		 //suprascrie cea mai veche valoare

                //ordonare dupa valoare

                    //elementul new se "scoate" din lista
                m_coada[m_new].prev->next = m_coada[m_new].next;			 //5.
                m_coada[m_new].next->prev = m_coada[m_new].prev;			 //6.

                    //update smallest value
                  structura* l_i = m_smallest;
                if (&(m_coada[m_new]) == m_smallest)
                {
                    if (m_coada[m_new].info > m_smallest->next->info)
                    {
                        m_smallest = m_smallest->next;
                        l_i = m_smallest;
                    }
                }
                else if (m_coada[m_new].info <= m_smallest->info)
                {
                    l_i = m_smallest;
                    m_smallest = &m_coada[m_new];
                }

                    //cautarea locului unde trebuie sa se amplaseze noul element in lista
                unsigned int l_cnt = 1;
                if (&(m_coada[m_new]) == l_i)
                {
                    l_i = l_i->next;
                }

                while ((l_i->info < m_coada[m_new].info) && (l_cnt <= m_size - 1))
                {
                    l_i = l_i->next;
                    l_cnt++;
                }

                    //inserarea elemntului new la locul potrivit
                l_i->prev->next = &m_coada[m_new];							 //1.
                m_coada[m_new].next = l_i;									 //2.
                m_coada[m_new].prev = l_i->prev;							 //3.
                l_i->prev = &m_coada[m_new];									 //4.

                //varianta ineficienta pentru aflarea medianului  cand filtrul are dimensiuni mari
                m_median=m_smallest;
                for(uint iddx=0; iddx < m_size/2; iddx++)
                {
                    m_median=m_median->next;
                }

                return getMedian();
            }

            template <class T, uint32_t N>
            T CMedianFilter<T, N>::getMedian()
            {
                T ret_val;
                if (1 == m_size % 2)		// daca filtrul are lungime impara
                {
                    ret_val = m_median->info;
                }
                else						// daca filtrul are lungime para
                {
                    ret_val = 0.5*(m_median->info + m_median->prev->info);
                }
                return ret_val;
            }
        }
    }
};

#include <filter.inl>
#endif // FILTER_H
