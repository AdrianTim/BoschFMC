#include <MotionController.h>

#define GAMMA 0.074255

 void CMotionController::_run()
    {
        if(m_splineinterpreter.isNewDataAvailable()){
            m_isSplineActivated=true;
            m_state=1;
            CSplineInterpreter::SplineInterpreter_Data spline=m_splineinterpreter.getData();
            m_motionPlanner.setMotionPlannerParameters(spline.isForward,spline.a,spline.b,spline.c,spline.d,spline.duration_sec,m_period_sec);
            // m_serialPort.printf("@SPLN:%d;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;;\r\n"
            //                         ,spline.isForward
            //                         ,spline.a.real()
            //                         ,spline.a.imag()
            //                         ,spline.b.real()
            //                         ,spline.b.imag()
            //                         ,spline.c.real()
            //                         ,spline.c.imag()
            //                         ,spline.d.real()
            //                         ,spline.d.imag()
            //                         ,spline.duration_sec);
        }
        if(m_isSplineActivated ){
            if(m_motionPlanner.hasValidValue()){
                std::pair<float,float> motion=m_motionPlanner.getNextVelocity();  
                float l_dir=m_motionPlanner.getForward()?1:-1;
                if(m_ispidActivated){
                    m_speed=motion.first*100.0*l_dir/2.0;
                    // m_serialPort.printf("@SPLN:%4.4f;%4.4f;;\r\n",m_speed,m_angle);
                }else{
                    m_speed=VEL2PWM(motion.first)*l_dir;
                    // m_serialPort.printf("@SPLN:%4.4f;%4.4f;;\r\n",m_speed,m_angle);
                }
                m_angle=motion.second;
                // m_serialPort.printf("@SPLN:%4.4f;%4.4f;;\r\n",m_speed,m_angle);
            }else{
                m_serialPort.printf("@SPLN:Stop;;\r\n");
                m_speed=0.0;
                m_state=2;
                m_isSplineActivated=false;
                

            }
        }
        
        //verificare safetystop
        //safety stop function
        if ((m_state!=2)&&(m_safetyStop.isSafetyStopActive(m_speed,m_angle)==true)){
            m_state = 2;
        }
        switch(m_state){
            // Moving 
            case 1:
                m_car.Steer(m_angle);
                if(m_ispidActivated){
                    // *0.430875709375
                    // m_serialPort.printf("MCTL:%4.4f;%4.4f;;\r\n",m_speed,m_angle);
                    m_control.setRef(m_speed*0.3804);//U
                    m_control.control();//calculate -U->Y
                    m_car.Speed(m_control.get());//Y
                }
                else{
                    if(m_speed<20 && m_speed>-20){
                        m_car.Speed(m_speed);
                    }
                }
                break;
            // Braking 
            case 2:
                m_car.Steer(m_angle);
                m_car.Brake();
                m_control.clear();
                break;
        }
    }

float CMotionController::VEL2PWM(float vel){
    return (vel/GAMMA)*0.41461003105583522+4.9068500777251405;
}
