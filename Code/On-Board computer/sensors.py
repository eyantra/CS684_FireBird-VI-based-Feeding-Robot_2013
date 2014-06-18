#Sensor_data
import struct
import binascii

#To senses the value of sensor (sensor_number) and return the value in integer
def sense_ir(ser,sensor_number):
	temp="020%s" %sensor_number
	inpt="NEX" + binascii.unhexlify(temp)
	ser.write(inpt)
	output=ser.read(6) 
	output=struct.unpack('i','%s\x00\x00\x00'%output[5])
	return output[0]
	
#senses all sensor and return an array of size 8 representing the sensor values
def sense_all_ir(ser):
	inpt="NEX\x05\x02" 
	ser.write(inpt)
	output=ser.read(13)
	return unpack_values(output)


#Turn on IR sensor
def on_ir(ser):
	inpt="NEX\x09\x01" 
	return ser.write(inpt)
		
#Turn off IR sensor
def off_ir(ser):
	inpt="NEX\x09\x02" 
	return ser.write(inpt)
			
#---------------------white line sensor datas---------------------
#To senses the value of sensor (sensor_number) and return the value in integer
def sense_line(ser,sensor_number):
	temp="040%s" %sensor_number
	inpt="NEX" + binascii.unhexlify(temp)
	ser.write(inpt)
	output=ser.read(6) 
	output=struct.unpack('i','%s\x00\x00\x00'%output[5])
	return output[0]
	
#senses all sensor and return an array of size 8 representing the sensor values
def sense_all_white_line(ser):
	inpt="NEX\x05\x04" 
	ser.write(inpt)
	output=ser.read(13)
	return unpack_values(output)
	
#Turn on white line sensor
def on_white_line(ser):
	inpt="NEX\x09\x03" 
	return ser.write(inpt)
		
#Turn off white line sensor
def off_white_line(ser):
	inpt="NEX\x09\x04" 	
	return ser.write(inpt)
	
#Get status of white line sensor
def get_white_line_status(ser):
	inpt="NEX\x0A\x02"
	ser.write(inpt)
	output=ser.read(6) 
	output=struct.unpack('i','%s\x00\x00\x00'%output[5])
	return output[0]
#------------------common---------------

def unpack_values(output):
	temp =[]
	i=0
	for i in range(5,13):
		temp1=struct.unpack('i','%s\x00\x00\x00'%output[i])
		temp.append(temp1[0])
	return temp
