
#include <mbed.h>
#include <Queue.h>
#include <Timer/Timer.h>
#include <commandInterpreter.h>
#include <TaskManager.h>
#include <MOVE.h>
#include <string>
#include <sstream>
#include <L3GD20H.h>
#include <LSM303D.h>
#include <IMU.h>
#include <Blinker.h>
#include <Echoer.h>
#include <map>
#include <array>
#include <SerialMonitor.h>
#include <MotionController.h>
#include <SplineInterpreter.h>
#include <EncoderFilter.h>
#include <sensortask.h>
#include <Controller.h>
#include <sensorpublisher.h>
#include <SafetyStopFunction.h>

Serial          g_rpi(USBTX, USBRX); 
MOVE            g_car(D9, D3, D2, D4, A0);
AnalogIn        currentSense(A2);
LSM303D         g_accelerometerMagnetometer(D14,D15,LSM303D::ACC_PREC_E::ACC_2G);
L3GD20H         g_gyroscope(D14,D15);


CTimer_100us    timer_100us;
const float g_baseTick = 0.0001; // seconds
CBlinker        g_blinker       (0.5    / g_baseTick, LED1);
CEchoer         g_echoer        (10     / g_baseTick, g_rpi);
CIMU            g_imu           (0.01   / g_baseTick, g_accelerometerMagnetometer, g_gyroscope);
float           g_period_Encoder=0.025;

filter::lti::siso::CIIRFilter<float,1,2> g_encoderFilter(linalg::CRowVector<float,1>({-0.72972973}) //A
                                                        ,linalg::CRowVector<float,2>({ 0.13513514  , 0.13513514  })); //B
CEncoderFilter  g_opticalEncoder(g_period_Encoder   / g_baseTick, g_period_Encoder, 40, D10,g_encoderFilter);                                                    
// controller::siso::CPidController<double> g_pid(0.32465*3.0,1.0221*3.0,0.014343*3.0,0.0023102,g_period_Encoder);
// controller::siso::CPidController<double> g_pid(9.5096,6.042,0.000,0.000,g_period_Encoder);
controller::siso::CPidController<double> g_pid(1.117284,1.03032,0.442,0.01887502,g_period_Encoder);

CSplineInterpreter      g_splineInterpreter;
filter::nonlinear::siso::CMedianFilter<float,21> g_filter1;
filter::nonlinear::siso::CMedianFilter<float,21> g_filter2;
filter::nonlinear::siso::CMedianFilter<float,21> g_filter3;
filter::nonlinear::siso::CMedianFilter<float,21> g_filter4;
filter::nonlinear::siso::CMedianFilter<float,21> g_filter5;
filter::nonlinear::siso::CMedianFilter<float,21> g_filter6;
CDriverTask<float> g_SenzStgFTask(0.05/g_baseTick, A1, g_filter1, g_rpi);
CDriverTask<float> g_SenzMiFSTask(0.05/g_baseTick, A5, g_filter2, g_rpi);
CDriverTask<float> g_SenzMiFDTask(0.05/g_baseTick, A3, g_filter3, g_rpi);
CDriverTask<float> g_SenzDrFTask(0.05/g_baseTick, A4, g_filter4, g_rpi);

CDriverTask<float> g_SenzStgSTask(0.05/g_baseTick, D11, g_filter5, g_rpi);
CDriverTask<float> g_SenzDrSTask(0.05/g_baseTick, D12, g_filter6, g_rpi);


CSafetyStopFunction         g_safetyStop( g_SenzStgFTask, g_SenzMiFSTask, g_SenzMiFDTask, g_SenzDrFTask, g_SenzStgSTask, g_SenzDrSTask);
std::array<CDriverTask<float>*,PROXIMITYPUBLISHER_NR> g_sensors={&g_SenzStgFTask,&g_SenzMiFSTask,&g_SenzMiFDTask,&g_SenzDrFTask,&g_SenzStgSTask};
sensors::ProximityPublisher                           g_sensorPub(0.05/g_baseTick,g_sensors,g_rpi);

task::CControl              g_controller(g_opticalEncoder,g_pid);
CMotionController           g_motionController(g_period_Encoder / g_baseTick, g_rpi, g_imu, g_car,g_splineInterpreter,0.01, g_safetyStop,g_controller);
examples::CEncoderSender    g_encoderSender(0.025   / g_baseTick,g_opticalEncoder,g_controller,g_rpi);

CSerialMonitor::CSerialSubscriberMap g_serialMonitorSubscribers = {
    {"MCTL",mbed::callback(CMotionController::staticSerialCallback,&g_motionController)},
    {"BRAK",mbed::callback(CMotionController::staticSerialCallbackBrake,&g_motionController)},
    {"HBRA",mbed::callback(CMotionController::staticSerialCallbackHardBrake,&g_motionController)},
    {"PIDA",mbed::callback(CMotionController::staticSerialCallbackPID,&g_motionController)},
    {"SPLN",mbed::callback(CSplineInterpreter::staticSerialCallback,&g_splineInterpreter)},
    {"SFBR",mbed::callback(CSafetyStopFunction::staticSerialCallback,&g_safetyStop)},
    {"PIDS",mbed::callback(controller::siso::CPidController<double>::staticSerialCallback,&g_pid)},
    {"DSPB",mbed::callback(sensors::ProximityPublisher::staticSerialCallback,&g_sensorPub)},
    {"ENPB",mbed::callback(examples::CEncoderSender::staticSerialCallback,&g_encoderSender)}
};

CSerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);


CTask* g_taskList[] = {
    &g_blinker,
    &g_opticalEncoder,
    &g_imu,
    &g_motionController,
    &g_serialMonitor,
    &g_encoderSender,
    &g_sensorPub,
    // &g_echoer
    &g_SenzStgFTask,
    &g_SenzMiFSTask,
    &g_SenzMiFDTask,
    &g_SenzDrFTask,
    &g_SenzStgSTask, 
    &g_SenzDrSTask,
}; 

CTaskManager g_taskManager(g_taskList, sizeof(g_taskList)/sizeof(CTask*), g_baseTick);

uint32_t setup()
{
    g_rpi.baud(460800);  
    g_rpi.printf("\r\n\r\n");
    g_rpi.printf("#################\r\n");
    g_rpi.printf("#               #\r\n");
    g_rpi.printf("#   I'm alive   #\r\n");
    g_rpi.printf("#               #\r\n");
    g_rpi.printf("#################\r\n");
    g_rpi.printf("\r\n");

    timer_100us.start();  
    return 0;    
}

uint32_t loop()
{
    g_taskManager.mainCallback();
    return 0;
}

int main() 
{
    uint32_t  l_errorLevel = setup(); 
    while(!l_errorLevel) 
    {
        l_errorLevel = loop();
    }
    g_rpi.printf("exiting with code: %d",l_errorLevel);
    return l_errorLevel;
}