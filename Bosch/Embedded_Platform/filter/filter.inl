template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
void filter::ltv::mimo::CEKF<T,NA,NB,NC>::predict(const CInputType&   f_input){
    //Previous updated state
    CStatesType l_prev_states=m_systemModel.getStates();
    CJMTransitionType l_JF=m_jbMatrices.getJMTransition(l_prev_states,f_input);
    //Predicted state estimate X_{k|k-1}
    CStatesType l_pred_states=m_systemModel.update(f_input);
    //Predicted covariance estimate
    m_covarianceMatrix=l_JF*m_covarianceMatrix*l_JF.transpose()+m_Q;
}

template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
void filter::ltv::mimo::CEKF<T,NA,NB,NC>::update(  const CInputType&   f_input
                                                    ,const COutputType&  f_measurment){
    // Estimated system output
    COutputType l_y_est=m_systemModel.calculateOutput(f_input);
    // Innovation or measurement residual
    COutputType l_mes_res=f_measurment-l_y_est;
    // Innovation (or residual) covariance
    CStatesType l_states=m_systemModel.getStates();
    CJMObservationType l_JH=m_jbMatrices.getJMObservation(f_input,l_states);
    CObservationNoiseType l_s=l_JH*m_covarianceMatrix*l_JH.transpose()+m_R;
    //Near-optimal Kalman gain
    CKalmanGainType l_K=m_covarianceMatrix*l_JH.transpose()*l_s.inv();
    //Updated state estimate
    CStatesType l_updated_states=l_states+l_K*l_mes_res;
    m_systemModel.setStates(l_updated_states);
    //Updated covariance estimate
    m_covarianceMatrix=(CJMTransitionType::eye()-l_K*l_JH)*m_covarianceMatrix;
}