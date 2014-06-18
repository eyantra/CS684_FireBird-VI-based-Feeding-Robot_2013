# program to follow white line

from time import sleep
import opencv_fun 
import connection
import sensors
import movements

P_W_L = 230		#left margin for plant detection
P_W_R = 285		#right margin for plant detection

def whiteline(plant_clear):
	THRESHOLD= 80
	FWD_FAST = "50"
	FWD_SLOW = "70"
	FWD_VSLOW = "90"

	FWD_FAST1 = "70"
	FWD_SLOW1 = "90"
	FWD_VSLOW1 = "110"

	STOP = "80"
	Loop_count = 2
	i=0
	

	sleep(0.3)
	ser = connection.connect()					#connect to FB 6
	movements.set_motor_mode(ser,1)				#set motor mode to 1
	sensors.on_white_line(ser)					#on whiteline sensors
	sleep(0.2)
	colour_tracker = opencv_fun.ColourTracker(0)
	
	loop_value = True
	value = 0
	
	

	while loop_value:
		
		#----for skiping detected plant location if any ---#
		i=i+1
		if plant_clear and i==Loop_count:
			loop_value=False
		else:
			loop_value = True
		#sleep(0.1)
		#---------------------------------------------------#
		
		Whiteline = sensors.sense_all_white_line(ser)
		(plant,pos) = plantFound(colour_tracker)	#get plant status
		#print Whiteline

		if plant == 2 or plant_clear:
			if((Whiteline[3]<=THRESHOLD) or (Whiteline[4]<=THRESHOLD)) :
				if value == 1:						#to avoid repeating the same commend
					print "Same"
				else:
					Left_Velocity = FWD_FAST
					Right_Velocity = FWD_FAST
					movements.move_bot(ser,Left_Velocity,Right_Velocity)
					print "Go Stright[3,4]"
					value=1
	 
			elif((Whiteline[5]< THRESHOLD) or (Whiteline[6] < THRESHOLD)):
				if value == 2:						#to avoid repeating the same commend
					print "Same"
				else:
					Left_Velocity = FWD_FAST
					Right_Velocity = FWD_VSLOW 
					movements.move_bot(ser,Left_Velocity,Right_Velocity)
					print "Turn Right"
					value = 2
	
			elif(( Whiteline[2] < THRESHOLD) or (Whiteline[1] < THRESHOLD)):
				if value == 3:						#to avoid repeating the same commend
					print "Same"

				else:
					Left_Velocity = FWD_VSLOW
					Right_Velocity = FWD_FAST 
					movements.move_bot(ser,Left_Velocity,Right_Velocity)
					print "Turn LEFT"
					value = 3
			else:
				if value == 4:						#to avoid repeating the same commend
					print "Same"
				else:
					Left_Velocity = STOP
					Right_Velocity = STOP
					movements.stop_here(ser)
					value = 4

		
		Whiteline = sensors.sense_all_white_line(ser)
		(plant,pos) = plantFound(colour_tracker)	#get plant status and possition

		if plant == 1 and not plant_clear:
			movements.stop_here(ser)
			Left_Velocity = FWD_VSLOW
			Right_Velocity = FWD_VSLOW
			#sleep(1)
			(plant,pos) = plantFound(colour_tracker)
			print "Plant red d0t %s" %plant
			if pos < P_W_L:
				print "plant < P_W_L: Move <-"
				movements.move_bot(ser,Left_Velocity,Right_Velocity)
				#sleep(0.5)
				movements.stop_here(ser)
				
			elif pos >P_W_R:
				print "plant < P_W_L: Move ->"
				movements.move_bot_back(ser,Left_Velocity,Right_Velocity)
				#sleep(0.5)
				movements.stop_here(ser)
				movements.clear_encode_counter(ser)
			

			#(plant,pos) = plantFound(colour_tracker)
			#sleep(1)
			if (plant != 0):
				plant = find_plant(ser,colour_tracker,plant,pos)
				
			value =5
			
			
			'''#sleep(0.3)
			(plant,pos) = plantFound(colour_tracker)	#get plant status
			if((Whiteline[3]<=THRESHOLD) or (Whiteline[4]<=THRESHOLD)) :
				Left_Velocity = FWD_FAST1
				Right_Velocity = FWD_FAST1
				movements.move_bot(ser,Left_Velocity,Right_Velocity)
				print "Go Stright[3,4]"
	 
			elif((Whiteline[5]< THRESHOLD) or (Whiteline[6] < THRESHOLD)):
				Left_Velocity = FWD_FAST1
				Right_Velocity = FWD_VSLOW1
				movements.move_bot(ser,Left_Velocity,Right_Velocity)
				print "Turn Right"
	
			elif(( Whiteline[2] < THRESHOLD) or (Whiteline[1] < THRESHOLD)):
				Left_Velocity = FWD_VSLOW1
				Right_Velocity = FWD_FAST1
				movements.move_bot(ser,Left_Velocity,Right_Velocity)
				print "Turn LEFT"
			else:
				Left_Velocity = STOP
				Right_Velocity = STOP
				movements.stop_here(ser)
			print "Plant red det %s" %plant
			movements.stop_here(ser)
			#sleep(0.3)
			(plant,pos) = plantFound(colour_tracker)'''

		#Whiteline = sensors.sense_all_white_line(ser)
		#(plant,pos) = plantFound(colour_tracker)	#get plant status		

		if plant == 0 and not plant_clear:
			print "++++plant+++"
			movements.stop_here(ser)
			movements.set_motor_mode(ser,2)
			movements.set_position_one_b(ser)
			value =6
			#plant == 1
			#find_plant(ser,colour_tracker)
			return
		print "-----------------------------"
		
#function for plant detection
def plantFound(colour_tracker):
	
	ret = colour_tracker.show3()
	print ret
	
	if ret > P_W_L and ret < P_W_R:   # plant found
		print "Plant ---Found---"
		return (0 ,ret)
	elif ret != -1:
		return (1,ret)
	else:
		print "Plant ---NOT Found---"
		return (2 ,ret)


#funtion to align the robot to the center

def find_plant(ser,colour_tracker,plant,pos):
	print "<-finder->"
	#sleep(1)
	#(plant, pos) = plantFound(colour_tracker)
	movements.clear_encode_counter(ser)
	movements.set_motor_mode(ser,2)
	
	while plant != 0 and plant != 2:
		print "---%d---"%plant
		print movements.get_motor_mode(ser)
		if pos < P_W_L:
			print "finder-> )"
			movements.set_position_one_f(ser)
			#sleep(1)
			
		elif pos > P_W_R:
			print "<-finder"   #error may occur in this place
		        # movements.clear_encode_counter(ser)
			movements.set_position_one_b(ser)
			sleep(0.01)
		(plant,pos) = plantFound(colour_tracker)
	value =6
	return plant
		
	
'''
def find_plant(ser,colour_tracker):
	print "<-finder->"
	#sleep(1)
	(plant,pos) = plantFound(colour_tracker)
	movements.clear_encode_counter(ser)
	movements.set_motor_mode(ser,2)
	Done = False
	while plant != 0 or plant != 2:
		movements.clear_encode_counter(ser)
		count = movements.right_motor_encoder_count(ser)
		if plant < P_W_L:
			print "finder-> movements.set_position(ser,10,90,10,90)"
			movements.set_position(ser,10,90,10,90)
			while count < 16:
				count = movements.right_motor_encoder_count(ser)
				print count
		if plant > P_W_R:
			print "<-finder movements.set_position(ser,10,70,10,70)"
			movements.set_position(ser,10,70,10,70)
			while count < 16:
				count = movements.right_motor_encoder_count(ser)
				print count
			#sleep(2)
		(plant,pos) = plantFound(colour_tracker)
	return
		
elif(Whiteline[3]>=THRESHOLD):
		Left_Velocity = "5f";
		Right_Velocity = "50" ;
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn Right[3]"
		
	elif(Whiteline[4]>=THRESHOLD):
		Left_Velocity = "50";
		Right_Velocity = "5f";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Trun Left[4]"
		
	elif(Whiteline[2]<=THRESHOLD):
		Left_Velocity = "50";
		Right_Velocity = "5f";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE Left [2]"
		
	elif(Whiteline[1]<=THRESHOLD):
		Left_Velocity = "50";
		Right_Velocity = "6e";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE Left [1]"
		
	elif(Whiteline[0]<=THRESHOLD):
		Left_Velocity = "50";
		Right_Velocity = "78";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE Left [0]"
		
	elif(Whiteline[5]<=THRESHOLD):
		Left_Velocity = "5f";
		Right_Velocity = "50";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE Right [5]"
		
	elif(Whiteline[6]<=THRESHOLD):
		Left_Velocity = "6e";
		Right_Velocity = "50";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE Right [6]"
		
	elif(Whiteline[7]<=THRESHOLD):
		Left_Velocity = "78";
		Right_Velocity = "50";
		movements.move_bot(ser,Left_Velocity,Right_Velocity);
		print "Turn LARGE Right [7]"
	'''	
		
