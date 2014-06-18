#conneting FB VI in serial
import serial
'''This program is to connect the host i.e onboard computer
   to the fire bird VI robot and atmega32 kit via serial ports '''

#connection setup for FB VI
ser=serial.Serial()
ser.port='/dev/ttyUSB1'	#48 is the (COM port number - 1)
ser.baudrate=9600

#connection setup for atmega32 board
ser_ext=serial.Serial()
ser_ext.port='/dev/ttyUSB0'	#48 is the (COM port number - 1)
ser_ext.baudrate=9600

def connect():	 		#connect FB 6
	ser.open()
	return ser

def ext_connect():		#connect Atmega32 board
	ser_ext.open()
	return ser_ext

def disconnect(ser): 	#disconnect serial 
	return ser.close()

def reset(ser):			#reset the connection
	ser.close()
	return ser.open()
	
	
