from threading import Thread
from socket import *
from math import *
from random import *
from time import *
import os
import sys

THIS_CAR_ID 				= 1
gw=os.popen("ip -4 route show default").read().split()
s=socket(AF_INET,SOCK_DGRAM)
s.connect((gw[2],0))
HOST_NAME=gethostname()
HOST_IP=s.getsockname()[0]
s.close()
print("Host name:"+str(HOST_NAME)+" IP:"+str(HOST_IP))

NEGOTIATION_PORT    		= 12346
CAR_SUBSCRITPION_PORT   	= NEGOTIATION_PORT + 2
CAR_COMMUNICATION_PORT 		= CAR_SUBSCRITPION_PORT + 2
MAX_WAIT_TIME_FOR_SERVER 	= 10
SERVER_IP 					= None
car_id 						= THIS_CAR_ID
car_poz 					= 0+0j
car_or 						= 0+0j
NEW_SERVER_IP				= False
START_UP					= True
G_Socket_Poz				= socket()
ID_SENT_FLAG				= False
RUN_CARCLIENT				= True

#------------------------------------------------------------------------
def GetServer():
	# listen for server broadcast and extract server IP address
	global SERVER_IP
	global NEGOTIATION_PORT
	global NEW_SERVER_IP
	global G_Socket_Poz
	global START_UP
	global ID_SENT_FLAG
	global RUN_CARCLIENT
	
	while RUN_CARCLIENT:
		try:
			s = socket(AF_INET, SOCK_DGRAM)
			s.bind(('', NEGOTIATION_PORT))
			s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

			WAIT_TIME = 2 + floor(10*MAX_WAIT_TIME_FOR_SERVER*random())/10
			#print("Waiting " + str(WAIT_TIME) + " seconds for server" )
			t = time()
			server = []

			# listen for server broadcast
			s.settimeout(WAIT_TIME) 
			data, server_ip = s.recvfrom(1500, 0)
			if server_ip[0] != SERVER_IP:
				# new server
				SendIDToServer(server_ip[0])	
				NEW_SERVER_IP = True
				SERVER_IP = server_ip[0] # server is alive
				START_UP = False		
			else:
				# old server
				if ID_SENT_FLAG == False:
					SendIDToServer(server_ip[0])
					print("Subscribe @ GPS server")
				NEW_SERVER_IP = False			
			# server beacon received
			s.close()
			
		except Exception as e:
			SERVER_IP = None # server is dead
			if START_UP == False and SERVER_IP == None and NEW_SERVER_IP == True:
				G_Socket_Poz.close()
				G_Socket_Poz=None
				print("Socket from get position closed!")
				
			print ("Not connected to server! IP: " + str(SERVER_IP) + "! Error:" + str(e))
			s.close()

#------------------------------------------------------------------------
def SendIDToServer(new_server_ip):
    #sends THIS_CAR_ID to server for identification
	global SERVER_IP
	global CAR_SUBSCRITPION_PORT
	global THIS_CAR_ID
	global ID_SENT_FLAG

	try:
		s = socket()         
		print("Vehicle " + str(THIS_CAR_ID) + " subscribing to GPS server: " + str(new_server_ip) +":"+ str(CAR_SUBSCRITPION_PORT))
		s.connect((new_server_ip, CAR_SUBSCRITPION_PORT))
		sleep(2) 
		car_id_str = str(THIS_CAR_ID)
		s.send(bytes(car_id_str, "UTF-8"))
		s.shutdown(SHUT_WR)
		s.close()
		print("Vehicle ID sent to server----------------------------")
		ID_SENT_FLAG = True
	except Exception as e:
		print("Failed to send ID to server, with error: " + str(e))
		s.close()
		ID_SENT_FLAG = False
		
#------------------------------------------------------------------------
def GetPositionDataFromServer_Thread():
	#receive car position and orientation from server
	global car_id
	global car_poz
	global car_or
	global SERVER_IP
	global NEW_SERVER_IP
	global THIS_CAR_ID
	global CAR_COMMUNICATION_PORT
	global G_Socket_Poz
	global RUN_CARCLIENT

	while RUN_CARCLIENT:
		if SERVER_IP != None:	# if server alive/valid
			if NEW_SERVER_IP == True:
				try:
					G_Socket_Poz.close()
					G_Socket_Poz=None
				except:
					# do nothing
					print("do nothing ...")
			if G_Socket_Poz==None:
				try:
					# if there is a GPS server available then open a socket and then wait for GPS data
					print("New socket was created to receive the position from the serves of:"+str(HOST_IP)+":"+str(CAR_COMMUNICATION_PORT))
					G_Socket_Poz = socket()      				
					G_Socket_Poz.bind((HOST_IP, CAR_COMMUNICATION_PORT))      
					G_Socket_Poz.listen(2)	
					NEW_SERVER_IP=False	
				except Exception as e:
					print("Creating new socket for get position from server: " + str(SERVER_IP) + " with error: "+str(e))
					sleep(1)
					G_Socket_Poz=None		
			if not G_Socket_Poz==None:
				try:
					c, addr = G_Socket_Poz.accept()
					data = str(c.recv(4096)) # raw message      
					# mesage parsing
					car_id= int(data.split(';')[0].split(':')[1])
					if car_id == THIS_CAR_ID:
						car_poz = complex(float(data.split(';')[1].split(':')[1].split('(')[1].split('+')[0]), float(data.split(';')[1].split(':')[1].split('j')[0].split('+')[1]))
						car_or = complex(float(data.split(';')[2].split(':')[1].split('(')[1].split('+')[0]), float(data.split(';')[2].split(':')[1].split('j')[0].split('+')[1]))
						print("id:" + str(car_id) + " -position: " + str(car_poz) + " -orientation: " + str(car_or))
					c.close()
				except Exception as e:
						print("Receiving position data from server failed! from: " + str(SERVER_IP) + " with error: " + str(e))
						c.close()
	
		# [TO DO] validate received GPS data

#------------------------------------------------------------------------
def main():
	global RUN_CARCLIENT
	print("First start up of vehicle client ...")

	# thread responsible for subscribing to GPS server
	connection_thread = Thread(target = GetServer)

	# thread responsible for receiving GPS data from server
	position_thread = Thread(target = GetPositionDataFromServer_Thread)
	
	try:
		# start all threads
		connection_thread.start()
		position_thread.start()
		while RUN_CARCLIENT:
			print("Running")
			sleep(30)
	except KeyboardInterrupt as e:
		RUN_CARCLIENT=False
		try:
			global G_Socket_Poz
			G_Socket_Poz.close()
		except:
			print("Cannot close client socket!")
			pass
		print("KeyboardInterrupt!!")
		pass

    
if __name__ == "__main__":
	main()
