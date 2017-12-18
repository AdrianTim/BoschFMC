import SerialHandler
import threading,serial,time,sys
import SaveEncoder


global serialHandler
A=0.0j
B=0.33j
C=0.66j
D=1.0j
timer=2.0

def testBezier(serialHandler):
    #Event test
    ev1=threading.Event()
    serialHandler.readThread.addWaiter("SPLN",ev1)
    time.sleep(2.0)
    print("Start state0")
    sent=serialHandler.sendBezierCurve(A,B,C,D,timer,True)
    if sent:
        ev1.wait()
        print("Confirmed")
        ev1.clear()
        ev1.wait()
        print("Terminated")
    else:
        print("Sending problem")
    print("END_TEST")
    serialHandler.readThread.deleteWaiter("SPLN",ev1)
    time.sleep(0.0)
def testMOVEAndBrake(serialHandler):
    #Event test
    pwm=10.0
    e=SaveEncoder.SaveEncoder("EncoderPID%.2f"%pwm+".csv")
    e.open()
    
    ev1=threading.Event()
    ev2=threading.Event()
    serialHandler.readThread.addWaiter("MCTL",ev1,e.save)
    serialHandler.readThread.addWaiter("BRAK",ev1,e.save)
    serialHandler.readThread.addWaiter("ENPB",ev2,e.save)

    time.sleep(1.0)
    print("Start moving")
    sent=serialHandler.sendMove(pwm,0.0)
    if sent:
        ev1.wait()
        print("Confirmed")
        
    else:
        print("Sending problem")
    time.sleep(3.0)
    ev1.clear()
    print("Start braking")
    sent=serialHandler.sendBrake(0.0)
    if sent:
        ev1.wait()
        print("Confirmed")
        
    else:
        print("Sending problem")
    
    time.sleep(1.0)
    print("END_TEST")
    serialHandler.readThread.deleteWaiter("BRAK",ev1)
    serialHandler.readThread.deleteWaiter("MCTL",ev1)
    serialHandler.readThread.deleteWaiter("ENPB",ev2)
    e.close()
    
def testBrake(serialHandler):
    #Event test
    ev1=threading.Event()
    serialHandler.readThread.addWaiter("BRAK",ev1)
    time.sleep(2.0)


    print("Start moving")
    for i in range(0,8):
        if i%2==0:
            sent=serialHandler.sendBrake(-20.0)
        else:
            sent=serialHandler.sendBrake(20.0)
        if sent:
            ev1.wait()
            print("Confirmed")
            
        else:
            print("Sending problem")
        time.sleep(1)
        ev1.clear()
    
    time.sleep(1)
    # sent=serialHandler.sendBrake(0.0)
    if sent:
            ev1.wait()
            print("Confirmed")
    serialHandler.readThread.deleteWaiter("MCTL",ev1)
def testPid(SerialHandler):
    ev1=threading.Event()
    serialHandler.readThread.addWaiter("PIDA",ev1)
    sent=serialHandler.sendPidActivation(True)
    if sent:
        ev1.wait()
        print("Confirmed")
        
    else:
        print("Sending problem")
    serialHandler.readThread.deleteWaiter("PIDA",ev1)
def testSafetyStop(SerialHandler):
    ev1=threading.Event()
    serialHandler.readThread.addWaiter("SFBR",ev1)
    sent=serialHandler.sendSafetyStopActivation(False)
    if sent:
        ev1.wait()
        print("Confirmed")
        
    else:
        print("Sending problem")
    serialHandler.readThread.deleteWaiter("SFBR",ev1)
def testPidValue(SerialHandler):
    ev1=threading.Event()
    serialHandler.readThread.addWaiter("PIDS",ev1)
    # (kp,ki,kd,tf)=(2.8184,7.0832,0.28036,0)
    (kp,ki,kd,tf)=(0.93143,2.8,0.0,0.0001)
    sent=serialHandler.sendPidValue(kp,ki,kd,tf)
    if sent:
        ev1.wait()
        print("Confirmed")
        
    else:
        print("Sending problem")
    serialHandler.readThread.deleteWaiter("PIDS",ev1)

def main():
    #Initiliazation
    global serialHandler
    serialHandler=SerialHandler.SerialHandler()
    serialHandler.startReadThread()
    # testBezier(serialHandler)
    # testBrake(serialHandler)
    testMOVEAndBrake(serialHandler)
    time.sleep(1.0)
    serialHandler.close()

if __name__=="__main__":
    main()
        
