#ifndef SAFETY_STOP_FUNCTION_H
#define SAFETY_STOP_FUNCTION_H

#include <DriverTask.h>
#include <MotionController.h>

class CSafetyStopFunction{
public:
    CSafetyStopFunction(CDriverTask<float>& f_senzStgF
                        , CDriverTask<float>& f_senzMiSF
                        , CDriverTask<float>& f_senzMiDF
                        , CDriverTask<float>& f_senzDrF
                        , CDriverTask<float>& f_senzStgS
                        , CDriverTask<float>& f_senzDrS)
                        :m_active(false) 
                        ,m_viteza(0)
                        , m_unghi(0)
                        , m_senzStgF(f_senzStgF)
                        , m_senzMiSF(f_senzMiSF)
                        , m_senzMiDF(f_senzMiDF)
                        , m_senzDrF(f_senzDrF)
                        , m_senzStgS(f_senzStgS)
                        , m_senzDrS(f_senzDrS)
                        {
                        }
    static void staticSerialCallback(void* obj,char const * a, char * b)
    {
        CSafetyStopFunction* self = static_cast<CSafetyStopFunction*>(obj);
        self->serialCallback(a,b);
    }
    bool isSafetyStopActive(float f_viteza, float f_angle){
        if (!m_active){
            return false;
        }

        m_viteza = f_viteza;
        m_unghi = f_angle;
        bool ret_val;
        ret_val = false;

        
        float prag;
        if (m_viteza > 0)
        {
            prag = m_viteza*8-(1/m_viteza)*10;
            //prag=20;
            if (  ( m_senzStgF.getDistanceFiltered() <= prag) || (m_senzMiSF.getDistanceFiltered() <= prag) || (m_senzMiDF.getDistanceFiltered() <= prag) || (m_senzDrF.getDistanceFiltered() <= prag)  )
            {
                ret_val = true;
            }
        }
        else if (m_viteza < 0)
                {
                    prag = m_viteza*8+(1/m_viteza)*10;
                    if ( (m_senzStgS.getDistanceFiltered() <= prag*(-1)) || (m_senzDrS.getDistanceFiltered() <= prag*(-1))  )
                    {
                        ret_val = true;
                    }
                }
        return ret_val;
    }
private:
    void serialCallback(char const * a, char * b){
        int l_isActivate=0;
        uint32_t l_res= sscanf(a,"%d",&l_isActivate);
        if(l_res==1){
            m_active=(l_isActivate>=1);
            sprintf(b,"ack;;");
        }else{
            sprintf(b,"sintax error;;");
        }
    }

    bool  m_active;
    float m_viteza;
    float m_unghi;
    CDriverTask<float>& m_senzStgF;
    CDriverTask<float>& m_senzMiSF;
    CDriverTask<float>& m_senzMiDF;
    CDriverTask<float>& m_senzDrF;
    CDriverTask<float>& m_senzStgS;
    CDriverTask<float>& m_senzDrS;

};
#endif







