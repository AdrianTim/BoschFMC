import serial,sys, time
import threading


class MessageConverter:
    def MCTL(f_vel=0.0,f_angle=0.0):
        if type(f_vel==float) and type(f_angle==float):
            return "#MCTL:%.2f"%f_vel+";%.2f"%f_angle+";;\r\n"
        else:
            return ""
    def BRAKE(f_angle=0.0):
        if type(f_angle)==float:
            return "#BRAK:%.2f"%f_angle+";;\r\n"
        else:
            return ""
    def SPLN(A,B,C,D,dur_sec=1.0,isForward=True):
        if type(dur_sec)==float and type(isForward)==bool:
            isAllComplex=(type(A)==complex and type(B)==complex and type(C)==complex and type(D)==complex)
            isAllList=((type(A)==list and type(B)==list and type(C)==list and type(D)==list) and (len(A==2) and len(B==2) and len(C==2) and len(D==2)))
            isForward=int(isForward)
            if isAllComplex:
                return '#SPLN:%d;'%isForward+'%.2f;'%A.real+'%.2f;'%A.imag+'%.2f;'%B.real+'%.2f;'%B.imag+'%.2f;'%C.real+'%.2f;'%C.imag+'%.2f;'%D.real+'%.2f;'%D.imag+'%.2f;'%dur_sec+';\r\n'
            elif isAllList:
                return '#SPLN:%d;'%isForward+'%.2f;'%A[0]+'%.2f;'%A[1]+'%.2f;'%B[0]+'%.2f;'%B[1]+'%.2f;'%C[0]+'%.2f;'%C[1]+'%.2f;'%D[0]+'%.2f;'%D[1]+'%.2f;'%dur_sec+';\r\n'
            else:
                return ""
        else:
            return ""
    def PIDS(kp,ki,kd,tf):
        if type(kp)==float and type(ki)==float and type(kd)==float and type(tf)==float:    
            return "#PIDS:%.5f;"%kp+"%.5f;"%ki+"%.5f;"%kd+"%.5f;"%tf+";\r\n"
        else:
            return ""
    def PIDA(activate=True):
        l_value=0
        if activate:
            l_value=1
        return '#PIDA:%d;'%l_value+';\r\n'
    def SFBR(activate=True):
        l_value=0
        if activate:
            l_value=1
        return '#SFBR:%d;'%l_value+';\r\n'
    def ENPB(activate=True):
        l_value=0
        if activate:
            l_value=1
        return '#ENPB:%d;'%l_value+';\r\n'
    def DSPB(activate=True):
        l_value=0
        if activate:
            l_value=1
        return '#DSPB:%d;'%l_value+';\r\n'


class ReadThread(threading.Thread):
    class CallbackEvent:
        def __init__(self,event,callbackFunc):
            self.event=event
            self.callbackFunc=callbackFunc
    def __init__(self,f_theadID,f_serialCon,f_fileHandler,f_printOut=False):
        threading.Thread.__init__(self)
        self.ThreadID=f_theadID
        self.serialCon=f_serialCon
        self.fileHandler=f_fileHandler
        self.Run=False
        self.buff=""
        self.isResponse=False
        self.printOut=f_printOut
        self.Responses=[]
        self.Waiters={}
    def run(self):
        while(self.Run):
            if self.serialCon.inWaiting()>=1:
                read_chr=self.serialCon.read()
                try:
                    read_chr=(read_chr.decode("ascii"))
                    if read_chr=='@':
                        self.isResponse=True
                        if len(self.buff)!=0:
                            self.checkWaiters(self.buff)
                        self.buff=""
                    elif read_chr=='\r':
                        self.isResponse=False
                        if len(self.buff)!=0:
                            self.checkWaiters(self.buff)
                        self.buff=""
                    if self.isResponse:
                        self.buff+=read_chr
                    self.fileHandler.write(read_chr)    
                    if self.printOut:
                        sys.stdout.write(read_chr)
                except UnicodeDecodeError:
                    pass
    def checkWaiters(self,f_response):
        l_key=f_response[1:5]
        if l_key in self.Waiters:
            l_waiters=self.Waiters[l_key]
            for eventCallback in l_waiters:
                eventCallback.event.set()
                if not eventCallback.callbackFunc==None:
                    eventCallback.callbackFunc(f_response[6:-2])
    def addWaiter(self,f_key,f_objEvent,callbackFunction=None):
        l_evc=ReadThread.CallbackEvent(f_objEvent,callbackFunction)
        if f_key in self.Waiters:
            obj_events_a=self.Waiters[f_key]
            obj_events_a.append(l_evc)
        else:
            obj_events_a=[l_evc]
            self.Waiters[f_key]=obj_events_a
    def deleteWaiter(self,f_key,f_objEvent):
        if f_key in self.Waiters:
            l_obj_events_a=self.Waiters[f_key]
            for callbackEventObj in l_obj_events_a:
                if callbackEventObj.event==f_objEvent:
                    l_obj_events_a.remove(callbackEventObj)            
    def stop(self):
        self.Run=False
    def start(self):
        self.Run=True
        super(ReadThread,self).start()
    
class FileHandler:
    def __init__(self,f_fileName):
        self.outFile=open(f_fileName,'w')
        self.lock=threading.Lock()
    def write(self,f_str):
        self.lock.acquire()
        self.outFile.write(f_str)
        self.lock.release()        
    def close(self):
        self.outFile.close()

class SerialHandler:
    def __init__(self,f_device_File='/dev/ttyACM0',f_history_file='historyFile.txt'):
        self.serialCon=serial.Serial(f_device_File,460800,timeout=1)
        self.historyFile=FileHandler(f_history_file)
        self.readThread=ReadThread(1,self.serialCon,self.historyFile)
        self.lock=threading.Lock()
    def startReadThread(self):
        self.readThread.start()
    def send(self,msg):
        # self.historyFile.write(msg)
        self.lock.acquire()
        self.serialCon.write(msg.encode('ascii'))
        self.lock.release()

    def sendMove(self,f_vel,f_angle):
        str_msg=MessageConverter.MCTL(f_vel,f_angle)
        if str_msg!="":
            self.send(str_msg)
            return True
        else:
            return False
    def sendBrake(self,f_angle):
        str_msg=MessageConverter.BRAKE(f_angle)
        if str_msg!="":
            self.send(str_msg)
            return True
        else:
            return False
    def sendBezierCurve(self,f_A,f_B,f_C,f_D,f_dur_sec,isForward):
        str_msg=MessageConverter.SPLN(f_A,f_B,f_C,f_D,f_dur_sec,isForward)
        if str_msg!="":
            self.send(str_msg)
            return True
        else:
            return False
    def sendPidActivation(self,activate=True):
        str_msg=MessageConverter.PIDA(activate)
        if str_msg!="":
            self.send(str_msg)
            return True
        else:
            return False
    def sendPidValue(self,kp,ki,kd,tf):
        str_msg=MessageConverter.PIDS(kp,ki,kd,tf)
        if str_msg!="":
            self.send(str_msg)
            return True
        else:
            return False
    def sendSafetyStopActivation(self,activate=True):
        str_msg=MessageConverter.SFBR(activate)
        if str_msg!="":
            self.send(str_msg)
            return True
        else:
            return False
    def sendDistanceSensorsPublischerActivation(self,activate=True):
        str_msg=MessageConverter.DSPB(activate)
        if str_msg!="":
            self.send(str_msg)
            return True
        else:
            return False
    def sendEncoderActivation(self,activate=True):
        str_msg=MessageConverter.ENPB(activate)
        if str_msg!="":
            self.send(str_msg)
            return True
        else:
            return False
    def close(self):
        self.readThread.stop()
        self.serialCon.close()
        self.historyFile.close()

if __name__=="__main__":
    main()
        
        