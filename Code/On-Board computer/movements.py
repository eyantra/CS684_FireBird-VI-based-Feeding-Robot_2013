#movements
'''This file contains all comment requred to move the robot'''
import struct
import binascii

#set motor mode
def set_motor_mode(ser,mode):
	if mode == 0:
		return ser.write("NEX\x90\x00")
	elif mode == 1:
		return ser.write("NEX\x90\x01")
	elif mode == 2:
		return ser.write("NEX\x90\x02")
	else :
		print "There is no %s mode" %mode
		return

#get the motor mode
def get_motor_mode(ser):
	inpt="NEX\x91\x00" 
	ser.write(inpt)
	output=ser.read(5) 
	output=struct.unpack('i','%s\x00\x00\x00'%output[4])
	return output[0]
	
def disable_timeout():
	inpt="NEX\x38" 
	return ser.write(inpt)
	
def stop_a():
	inpt="NEX\x32\x08"
	ser.write(inpt)
	inpt="NEX\x31\x08"
	ser.write(inpt)
	return True 
	
#move forward
def move_forward(ser):
	inpt="NEX\x94\x01" 	
	return ser.write(inpt)
	
#move backward
def move_backward(ser):
	inpt="NEX\x94\x02" 
	return ser.write(inpt)
	
def turn_right(ser):
	inpt="NEX\x94\x04" 
	return ser.write(inpt)
	
def turn_left(ser):
	inpt="NEX\x94\x03" 
	return ser.write(inpt)
	
def stop_here(ser):
	inpt="NEX\x94\x06" 
	return ser.write(inpt)
	
def left_motor_encoder_count(ser):
	inpt="NEX\x92\x00" 
	ser.write(inpt)
	output=ser.read(9) 
	return unpack_encoder_count(output)

def right_motor_encoder_count(ser):
	inpt="NEX\x93\x00" 
	ser.write(inpt)
	output=ser.read(9) 
	return unpack_encoder_count(output)

def unpack_encoder_count(output):
	temp =[]
	i=0
	for i in range(5,9):
		temp1=struct.unpack('i','%s\x00\x00\x00'%output[i])
		temp.append(temp1[0])
	return temp
	
def clear_encode_counter(ser):
	inpt="NEX\x8c\x00" 
	return ser.write(inpt)


	
def set_position(ser,p1,v1,p2,v2):
	#print "%s %s %s %s" %(p1,v1,p2,v2)
	temp1 ="0000"
	temp1 =binascii.unhexlify(temp1)
	temp2 ="00"
	temp2 =binascii.unhexlify(temp2)
	temp3 ="%d"%p1
	temp3 =binascii.unhexlify(temp3)
	tempt ="%d"%v1
	v1 =binascii.unhexlify(tempt)
	temp4 ="0000"
	temp4 =binascii.unhexlify(temp4)
	temp5 ="00"
	temp5 =binascii.unhexlify(temp5)
	temp6 ="%d"%p1
	temp6 =binascii.unhexlify(temp6)
	temp7 ="%d"%v2
	v2 =binascii.unhexlify(temp7)	
	inpt="NEX" +  binascii.unhexlify("9c") + temp1 + temp2+temp3+v1+temp4+temp5+temp6+v2
	#print "%s" %inpt
	return ser.write(inpt)

def set_position_one_f(ser):
	inpt="NEX\x9c\x00\x00\x00\x03\x46\x00\x00\x00\x03\x46" 
	#print "%s" %inpt
	return ser.write(inpt)

def set_position_one_b(ser):
	inpt="NEX\x9c\x00\x00\x00\x03\x90\x00\x00\x00\x03\x90" 
	#print "%s" %inpt
	return ser.write(inpt)
	
def move_bot(ser,left_velocity,right_velocity):
	temp="95%s" %left_velocity
	inpt1="NEX" + binascii.unhexlify(temp)
	temp="96%s" %right_velocity
	inpt2="NEX" + binascii.unhexlify(temp)
	ser.write(inpt1)
	ser.write(inpt2)
	return move_forward(ser)

def move_bot_back(ser,left_velocity,right_velocity):
	temp="95%s" %left_velocity
	inpt1="NEX" + binascii.unhexlify(temp)
	temp="96%s" %right_velocity
	inpt2="NEX" + binascii.unhexlify(temp)
	ser.write(inpt1)
	ser.write(inpt2)
	return move_backward(ser)
	
	
'''
def move_away(ser,distance):
	left stable, right backwardmove
	move forward,
	right_forward_move

def move_near(ser,distance):
	right stable, left backwardmove
	move forward
	left stable, right backwardmove
	
'''
