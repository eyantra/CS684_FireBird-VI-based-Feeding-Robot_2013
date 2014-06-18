'''This program manages all the operations involved in feeding process.
	step 1: extends the arm to an initial distance
	step 2: do image processing to find root
	step 3: extend a little and doe image processing to find root
	step 4: repeat step 3 till root found or no more extension possible
	step 5: rotate the AR screw and count the fertilize grain
	step 6: stop the rotation when count reached
	step 7: retrive the arm
'''

import opencv_fun #file has operation related to opencv
import struct
import connection
# Fertilizing function
def do_fertilize(plant_count):
	
	s_count=0 
	ser=connection.ext_connect()
	opencv_module = opencv_fun.ColourTracker(1)
	count=0
	dist_move=0
	temp=0
	#initial movement of ram
	while temp in range(0,12):
		inpt="N\x01\x42\x02" 			# Move arm forward
                ser.write(inpt)
		output=ser.read(2)
		print "init %s"%output
		temp=temp+1
    #move arm till root found
	while not root_detected(opencv_module):
		inpt="N\x01\x42\x02" 			# Move arm forward
                ser.write(inpt)
		output=ser.read(2)
		print output
		if output=="NF":
			count=count+1
		else:
			s_count=s_count+1
			dist_move =dist_move +1
		if s_count== 25 or count == 3: # cannot move any more
			break

	name = 'p%s.png'%plant_count
	opencv_module.saveImage(name)		#takes picture
        if s_count== 25 or count != 3:  #stoped becaue of root found
		print "root Detected"
		count = 0		
		f_count = 0
		while True:
			#rotate
			inpt="N\x03\x38\x01" 		# rotate 
			ser.write(inpt)
			output=ser.read(2)
			if output =="NF":
				count=count+1
			else:
				f_count=f_count+1
			if f_count== 1 or count == 83: # f_count = fertilize grain count, count==83 rep empty container
				break
	else:
		print "root not detected"
	
	name = 'pr%s.png'%plant_count
	opencv_module.saveImage(name)		#take picture
	
	count = 0		
	s=0
	while True:
		inpt="N\x02\x32\x02" 	# retrive arm backward
		ser.write(inpt)
		output=ser.read(2)
		print output
		if output =="NF":
			count=count+1
		else:
			s = s+1
		if count == 3 or s == 20:
			break
	print "fertilizer done"

#function to detect plant root
def root_detected(opencv_module):		
	if(opencv_module.match('./template/t1.jpg')):
		return True
	elif(opencv_module.match('./template/t2.jpg')):
		return True
	else:
		return False
#more templates can be added

