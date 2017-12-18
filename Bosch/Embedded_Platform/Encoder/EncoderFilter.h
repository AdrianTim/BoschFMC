#ifndef ENCODER_FILTER_H
#define ENCODER_FILTER_H
#include <Encoder.h>
#include <filter.h>

class CEncoderFilter:public CEncoder{
    template <class T>
    using CFilterFunction = filter::CFilterFunction<T>;
    public:
        CEncoderFilter  (uint32_t                       f_period
                        ,float                          f_period_sec
                        ,uint32_t                       f_cpr
                        ,PinName                        f_pinName
                        ,CFilterFunction<float>&        f_filter)
                        :CEncoder(f_period,f_period_sec,f_cpr,f_pinName)
                        ,m_filter(f_filter)
        {
        }
    float    getRpsFiltered(){
        return m_filteredRps;
    }
    protected:
        virtual void _run(){
            float l_count=m_counter.getCount();
            m_counter.reset();
            m_rps=static_cast<float>(l_count)/m_period_sec/m_cpr;
            m_filteredRps=m_filter(m_rps);
        }


    private:
        CFilterFunction<float>&    m_filter;
        float                      m_filteredRps;

};

#endif