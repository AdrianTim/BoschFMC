#ifndef MAG_ENCODER_H
#define MAG_ENCODER_H

#include <mbed.h>
#include <TaskManager.h>

class CCounter{
    public:
        CCounter(PinName f_pin)
                :m_interrupt(f_pin)
                ,m_count(0)
        {
            m_interrupt.rise(mbed::callback(CCounter::staticIncrement,this));
            m_interrupt.fall(mbed::callback(CCounter::staticIncrement,this));
        }


        void reset(){
            m_count=0;
        }

        void increment(){
            m_count++;
        }

        int32_t getCount(){
            return m_count;
        }

        static void staticIncrement(void* obj){
            CCounter* self=static_cast<CCounter*>(obj);
            self->increment();
        }
    private:

        InterruptIn m_interrupt;
        volatile int32_t m_count;
};


class CEncoder:public CTask{
    public:
        CEncoder(    uint32_t    f_period
                        ,float      f_period_sec
                        ,uint32_t   f_cpr
                        ,PinName    f_pinName)
                        :CTask(f_period)
                        ,m_counter(f_pinName)
                        ,m_period_sec(f_period_sec)
                        ,m_cpr(f_cpr)
                        ,m_rps(0)
        {
        }

        float getRPS(){return m_rps;}
    protected:
        virtual void _run(){
            float l_count=m_counter.getCount();
            m_counter.reset();
            m_rps=l_count/m_period_sec/m_cpr;
        }

        CCounter        m_counter;
        const float     m_period_sec;
        const uint32_t  m_cpr;//count per revolution (rise and fall)
        float           m_rps;

};


class CMagEncoderTime{
      public:
        CMagEncoderTime(PinName     f_pin)
                    :m_interrupt(f_pin)
        {
            m_interrupt.rise(mbed::callback(CMagEncoderTime::staticRise,this));
            // m_interrupt.fall(mbed::callback(CMagEncoderTime::staticFall,this));
            m_Timer.start();
        }

        void riseCallback(){
            m_Timer.stop();
            m_lowTime=m_Timer.read();
            m_Timer.reset();
            m_Timer.start();
        }

        void fallCallback(){
            m_Timer.stop();
            m_highTime=m_Timer.read();
            m_Timer.reset();
            m_Timer.start();
        }

        static void staticRise(void* obj){
            CMagEncoderTime* self=static_cast<CMagEncoderTime*>(obj);
            self->riseCallback();
        }

        static void staticFall(void* obj){
            CMagEncoderTime* self=static_cast<CMagEncoderTime*>(obj);
            self->fallCallback();
        }
        float getHighTime(){return m_highTime;}
        float getLowTime(){return m_lowTime;}
    private:
        Timer       m_Timer,m_fallTimer;
        InterruptIn m_interrupt;
        volatile float      m_highTime;
        volatile float      m_lowTime;

};
#endif