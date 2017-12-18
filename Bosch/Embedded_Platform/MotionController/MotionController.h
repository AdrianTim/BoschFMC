#ifndef MOTION_CONTROLLER_H 
#define MOTION_CONTROLLER_H 

#include <mbed.h>
#include <TaskManager.h>
#include <IMU.h>
#include <MOVE.h>
#include <SplineInterpreter.h>
#include <BezierMotionPlanner.h>
#include <Controller.h>
#include <SafetyStopFunction.h>

class CMotionController : public CTask
{
public:
    CMotionController(uint32_t f_period, Serial& f_serialPort, CIMU& f_imu, MOVE& f_car,CSplineInterpreter& f_splineInterpreter,float f_period_sec, CSafetyStopFunction& f_safetyStop,task::CControl& f_control) : CTask(f_period)
    , m_serialPort(f_serialPort)
    , m_imu(f_imu)
    , m_car(f_car)
    , m_speed()
    , m_angle()
    , m_period_sec(f_period_sec)
    , m_isSplineActivated(false)
    , m_ispidActivated(false)
    , m_splineinterpreter(f_splineInterpreter)
    , m_motionPlanner()
    , m_hbTimeOut()
    ,m_control(f_control)
    , m_safetyStop(f_safetyStop)
    {

    }
    static void staticSerialCallback(void* obj,char const * a, char * b)
    {
        CMotionController* self = static_cast<CMotionController*>(obj);
        self->serialCallback(a,b);
    }

    static void staticSerialCallbackBrake(void* obj,char const * a, char * b)
    {
        CMotionController* self = static_cast<CMotionController*>(obj);
        self->serialCallbackBrake(a,b);
    }

    static void staticSerialCallbackHardBrake(void* obj,char const * a, char * b) 
    {
        CMotionController* self = static_cast<CMotionController*>(obj);
        self->serialCallbackHardBrake(a,b);
    }
    static void staticSerialCallbackPID(void* obj,char const * a, char * b) 
    {
        CMotionController* self = static_cast<CMotionController*>(obj);
        self->serialCallbackPID(a,b);
    }
    void reset()
    {   
        m_speed = 0;
        m_angle = 0;
    }
    float getSpeed() 
    {
        return m_speed;
    }
    float getAngle() 
    {
        return m_angle;
    }

    void BrakeCallback(){
        m_state=2;
    }

    void setState(int f_state){
        m_state = f_state;
    }

private:
    virtual void _run();
    
    void serialCallback(char const * a, char * b)
    {
        float l_speed;
        float l_angle;
        uint32_t l_res = sscanf(a,"%f;%f",&l_speed,&l_angle);
        if (2 == l_res)
        {
            
            m_speed = l_speed;
            m_angle = l_angle; 
            m_isSplineActivated=false;
            m_state=1;
            sprintf(b,"ack;;");
        }
        else
        {
            sprintf(b,"sintax error;;");
        }
    }

    void serialCallbackBrake(char const * a, char * b){
        float l_angle;
        uint32_t l_res = sscanf(a,"%f",&l_angle);
        if(1 == l_res)
        {
            m_speed = 0;
            m_angle = l_angle; 
            m_state = 2;
            m_control.setRef(0);
            sprintf(b,"ack;;");           
        }else{
            sprintf(b,"sintax error;;");
        }
    }

    
    void serialCallbackHardBrake(char const * a, char * b){
        float l_brake,l_angle;
        uint32_t l_res = sscanf(a,"%f;%f",&l_brake,&l_angle);
        if(2 == l_res && m_state!=0)
        {
            m_speed=0;
            m_angle = l_angle; 
            m_car.Inverse(l_brake);
            m_hbTimeOut.attach(callback(this,&CMotionController::BrakeCallback),0.04);
            m_state=0;
            sprintf(b,"ack;;");           
        }else{
            sprintf(b,"sintax error;;");
        }
    }

    void serialCallbackPID(char const * a, char * b){
        int l_isActivate=0;
        uint32_t l_res = sscanf(a,"%d",&l_isActivate);
        if(l_res==1){
            m_ispidActivated=(l_isActivate>=1);
            sprintf(b,"ack;;");
        }else{
            sprintf(b,"sintax error;;");
        }
    }

    static float VEL2PWM(float vel);
    Serial& m_serialPort;
    CIMU& m_imu;
    MOVE& m_car;
    float m_speed;
    float m_angle;

    float   m_period_sec;
    bool    m_isSplineActivated;
    uint8_t m_state;
    bool    m_ispidActivated;
    // 0-none
    // 1-normal
    // 2-brake regeneration
    CSplineInterpreter&     m_splineinterpreter;
    CBezierMotionPlanner    m_motionPlanner;
    Timeout                 m_hbTimeOut;
    task::CControl&         m_control;
    CSafetyStopFunction&    m_safetyStop;
};

#endif // MOTION_CONTROLLER_H 
