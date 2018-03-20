import SerialHandler
import threading,serial,time,sys
import SaveEncoder
from SerialHandler import MessageConverter


'''
    EXAMPLE NR.1 
    The robot speed is controlled by the pwm signals without braking.The robot will move forward with a variable speed.
'''
def MoveWithPwmWithoutBrake(serialHandler):
    # Instances an event object, which will be unblocked, when the message was sent to Nucleo.
    sent_move=threading.Event()
    # Converting the action to the key word
    moveKeyStr=MessageConverter.MessageKeys.keyStr(MessageConverter.MessageKeys.MOVE)
    # Attach the event object to the key words. It will unblock the event object, after the response was received with the specified key word. 
    serialHandler.readThread.addWaiter(moveKeyStr,sent_move)
    # PWM signal to control the motor speed in percent [%], Limits are [-20,20]. 
    # If the speed value is negativ, than the robot will move backward. 
    # PWM signals 
    pwms=[10.0,10.5,11.0,11.5,12.0,11.5,11.0,10.5,10.0,0.0]
    #  Sleep duration in second after the sent message.
    sleep_duration=0.2
    for pwm in pwms:
        sent=serialHandler.sendMove(pwm,0.0)
        if sent:
            isConfirmed=sent_move.wait(timeout=1)
            if(isConfirmed):
                print("Response was received!")
            else:
                raise ConnectionError('Response', 'Response was not received!')
            sent_move.clear()
            time.sleep(sleep_duration)


    # Detach the event from the key word. If the event object was cleared and the wait member function is called without timeout, than the program will be blocked.
    serialHandler.readThread.deleteWaiter(moveKeyStr,sent_move)



'''
    EXAMPLE NR.2
    The robot speed is controlled by the pwm signals with braking. In this example, the robot will move forward and will brake. 
'''
def MoveWithPwmWithBrake(serialHandler):
    # Instances an event object, which will be unblocked, when the message was sent to Nucleo.
    sent_move=threading.Event()
    # Converting the action to the key word
    moveKeyStr=MessageConverter.MessageKeys.keyStr(MessageConverter.MessageKeys.MOVE)
    # Attach the event object to the key words. It will unblock the event object, after the response was received with the specified key word. 
    serialHandler.readThread.addWaiter(moveKeyStr,sent_move)
    # PWM signal to control the motor speed in percent [%], Limits are [-20,20]. 
    # If the speed value is negativ, than the robot will move backward. 
    # PWM signals 
    pwms=[10.0,10.5,11.0,11.5,12.0,11.5,11.0,10.5,10.0]
    #  Sleep duration in second after the sent message.
    sleep_duration=0.2
    steering_angle=0.0
    for pwm in pwms:
        # Send a move command with a motor speed [pwm signal in percent ] and a steering angle [degree]
        sent=serialHandler.sendMove(pwm,steering_angle)
        # 
        if sent:
            isConfirmed=sent_move.wait(timeout=1)
            if(isConfirmed):
                print("Response was received!")
            else:
                raise ConnectionError('Response', 'Response was not received!')
            sent_move.clear()
            time.sleep(sleep_duration)


    # Detach the event from the key word. If the event object was cleared and the wait member function is called without timeout, than the program will be blocked.
    serialHandler.readThread.deleteWaiter(moveKeyStr,sent_move)

    
    sent_brake=threading.Event()
    # Getting the braking action to the key word
    brakeKeyStr=MessageConverter.MessageKeys.BRAKE.value
    serialHandler.readThread.addWaiter(brakeKeyStr,sent_brake)
    # Send a braking command to the Nucleo with a steering angle [degree]
    sent=serialHandler.sendBrake(steering_angle)
    # If the sent variable has value true, than the command was sent. 
    if sent:
            isConfirmed=sent_brake.wait(timeout=1)
            if(isConfirmed):
                print("Response was received!")
            else:
                raise ConnectionError('Response', 'Response was not received!')
            sent_brake.clear()
    serialHandler.readThread.deleteWaiter(brakeKeyStr,sent_brake)


'''
    EXAMPLE NR.3
    The robot speed is controlled by the pwm signals with braking. In this example, we are saving all message received with 'ENPB', which is the encoder speed rotation.
    In this case, the speed will be controlled with negativ pwm values, and the angle will positive [rigth] in the moving state and will be negativ in the braking state.
'''
def MoveWithSaving(serialHandler):
    # Instances an object, which will save in the file all message recieved.
    saver=SaveEncoder.SaveEncoder("Encoder.csv")
    #  Open the file. 
    saver.open()

    encoder_event=threading.Event()
    encoderKeyStr=MessageConverter.MessageKeys.keyStr(MessageConverter.MessageKeys.ENCODER_ECHOER)
    # !!!!! In this case, is atteched an callback function to the word key, which will be called, when the message received. The input parameter for this callback function is the received message.
    serialHandler.readThread.addWaiter(encoderKeyStr,encoder_event,saver.save)

    # Active the encoder echoer to transmite the motor speed rotation
    sent=serialHandler.sendEncoderPublisher(True)
    if sent:
        isConfirmed=encoder_event.wait(timeout=1)
        if(isConfirmed):
            print('Confirmed the encoder activation!')
        else:
            raise ConnectionError('Response', 'Response was not received!')
        encoder_event.clear()

    # Instances an event object, which will be unblocked, when the message was sent to Nucleo.
    sent_move=threading.Event()
    # Converting the action to the key word
    moveKeyStr=MessageConverter.MessageKeys.keyStr(MessageConverter.MessageKeys.MOVE)
    # Attach the event object to the key words. It will unblock the event object, after the response was received with the specified key word.
    
    serialHandler.readThread.addWaiter(moveKeyStr,sent_move)
    # PWM signal to control the motor speed in percent [%], Limits are [-20,20]. 
    # If the speed value is negativ, than the robot will move backward. 
    # PWM signals
    pwms=[-10.0,-10.5,-11.0,-11.5,-12.0,-11.5,-11.0,-10.5,-10.0]
    #  Sleep duration in second after the sent message.
    sleep_duration=0.2
    # Steering angle positive -> rigth
    steering_angle=20.0
    for pwm in pwms:
        # Send a move command with a motor speed [pwm signal in percent ] and a steering angle [degree]
        sent=serialHandler.sendMove(pwm,steering_angle)
        # 
        if sent:
            isConfirmed=sent_move.wait(timeout=1)
            if(isConfirmed):
                print('Confirmed the moving command!')
            else:
                raise ConnectionError('Response', 'Response was not received!')
            
            sent_move.clear()
            time.sleep(sleep_duration)


    # Detach the event from the key word. If the event object was cleared and the wait member function is called without timeout, than the program will be blocked.
    serialHandler.readThread.deleteWaiter(moveKeyStr,sent_move)

    
    sent_brake=threading.Event()
    # Getting the braking action to the key word
    brakeKeyStr=MessageConverter.MessageKeys.BRAKE.value
    serialHandler.readThread.addWaiter(brakeKeyStr,sent_brake)
    # Steering angle negativ -> left
    steering_angle=-20.0
    # Send a braking command to the Nucleo with a steering angle [degree]
    sent=serialHandler.sendBrake(steering_angle)
    
    # If the sent variable has value true, than the command was sent. 
    if sent:
            isConfirmed=sent_brake.wait(timeout=1)
            if(isConfirmed):
                print('Confirmed the braking command!')
            else:
                raise ConnectionError('Response', 'Response was not received!')
            sent_brake.clear()
    serialHandler.readThread.deleteWaiter(brakeKeyStr,sent_brake)
    
    # Deactive the encoder echoer to transmite the motor speed rotation
    sent=serialHandler.sendEncoderPublisher(False)
    if sent:
        isConfirmed=encoder_event.wait(timeout=1)
        if(isConfirmed):
            print('Confirmed the encoder deactivation!')
        else:
            raise ConnectionError('Response', 'Response was not received!')
        encoder_event.clear()
    serialHandler.readThread.deleteWaiter(encoderKeyStr,encoder_event)
    #  Open the file.
    saver.close()


def MoveUsingPID(serialHandler):
    pid_activation_event=threading.Event()
    serialHandler.readThread.addWaiter(MessageConverter.MessageKeys.PID_ACTIVATE.value,pid_activation_event)
    # Activate PID
    sent=serialHandler.sendPidActivation(True)
    if sent:
        isConfirmed=pid_activation_event.wait(timeout=1)
        if(isConfirmed):
            print("Confirmed the PID activation commmand!")
        else:
            raise ConnectionError('Response', 'Response was not received!')
        pid_activation_event.clear()


    # Set the robot forward speed to the 30 cm/s 
    robot_speed=30 
    steering_angle=0.0
    #Seting the move duration in second 
    move_duration=3.0

    # Instances an event object, which will be unblocked, when the message was sent to Nucleo.
    sent_move=threading.Event()
    serialHandler.readThread.addWaiter(MessageConverter.MessageKeys.MOVE.value,sent_move)
    
    sent_brake=threading.Event()
    serialHandler.readThread.addWaiter(MessageConverter.MessageKeys.BRAKE.value,sent_brake)

    # Send a move command with a robot speed [cm/s] and a steering angle [degree]
    sent=serialHandler.sendMove(robot_speed,steering_angle)
    if sent:
        isConfirmed=sent_move.wait(timeout=1)
        if(isConfirmed):
            print('Confirmed the moving command!')
        else:
            raise ConnectionError('Response', 'Response was not received!')
        sent_move.clear()
    
    time.sleep(move_duration)

    # Send a brake command with a steering angle [degree]
    sent=serialHandler.sendBrake(steering_angle)
    if sent:
        isConfirmed=sent_brake.wait(timeout=1)
        if(isConfirmed):
            print('Confirmed the braking command!')
        else:
            raise ConnectionError('Response', 'Response was not received!')
        sent_brake.clear()

    # Deactivate PID
    sent=serialHandler.sendPidActivation(False)
    if sent:
        isConfirmed=pid_activation_event.wait(timeout=1)
        if(isConfirmed):
            print("Confirmed the PID activation commmand!")
        else:
            raise ConnectionError('Response', 'Response was not received!')
        pid_activation_event.clear()
    
    serialHandler.readThread.deleteWaiter(MessageConverter.MessageKeys.PID_ACTIVATE.value,pid_activation_event)
    serialHandler.readThread.deleteWaiter(MessageConverter.MessageKeys.MOVE.value,sent_move)
    serialHandler.readThread.deleteWaiter(MessageConverter.MessageKeys.BRAKE.value,sent_brake)

'''
    Main function
'''
def main():

    # Instances an object, which is used to communicate with Nucleo. The communication port by defualt is '/dev/ttyACM0' and the baud rate is 460800.
    serialHandler=SerialHandler.SerialHandler('COM10')
    # It starts the reader thread, which automatically saves the received messages from the Nucleo. This thread deactivates waiting state and it calls automatically a callback function.
    serialHandler.startReadThread()

    # Examples...
    MoveUsingPID(serialHandler)


    # Closes all port opened and all thread started.
    serialHandler.close()



if __name__=="__main__":
    main()