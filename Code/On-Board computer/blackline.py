'''Program to follow the black line'''

from time import sleep
import connection
import sensors
import movements

THRESHOLD=100    #threshold level for black line

sleep(0.3)
ser = connection.connect()		#connect to Firebird VI via serial port
movements.set_motor_mode(ser,1) #Set motor to mode 1 i.e. constant speed mode
sensors.on_white_line(ser)		#on whiteline sensors
sleep(2)

while True:
	sleep(0.1)
	Whiteline = sensors.sense_all_white_line(ser) #get whiteline data
	print Whiteline
	#robot in black line
	if(Whiteline[3]>=THRESHOLD and Whiteline[4]>=THRESHOLD) :
		Left_Velocity = "50";
		Right_Velocity = "50";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Go Stright"
		
	elif(Whiteline[3]>=THRESHOLD):
		Left_Velocity = "50";
		Right_Velocity = "5f";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn Right"
		
	elif(Whiteline[4]>=THRESHOLD):
		Left_Velocity = "5f";
		Right_Velocity = "50";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Trun Left"
		
	elif(Whiteline[2]>=THRESHOLD):
		Left_Velocity = "50";
		Right_Velocity = "5f";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE right [2]"
		
	elif(Whiteline[1]>=THRESHOLD):
		Left_Velocity = "50";
		Right_Velocity = "6e";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE right [1]"
		
	elif(Whiteline[0]>=THRESHOLD):
		Left_Velocity = "50";
		Right_Velocity = "78";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE right [0]"
		
	elif(Whiteline[5]>=THRESHOLD):
		Left_Velocity = "5f";
		Right_Velocity = "50";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE Left [5]"
	elif(Whiteline[6]>=THRESHOLD):
		Left_Velocity = "6e";
		Right_Velocity = "50";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE Left [6]"
		
	elif(Whiteline[7]>=THRESHOLD):
		Left_Velocity = "78";
		Right_Velocity = "50";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE Left [7]"
		
	else:
		Left_Velocity = "80";
		Right_Velocity = "80";
		movements.stop_here(ser);
		
