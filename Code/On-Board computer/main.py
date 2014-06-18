#!/usr/bin/python
'''This is the main program of the feeding robot
   step1 : follow the wihitlein and stops when blue spot found and align to center
   step2 : Fertilize the plant
    Step3 : Move forward by following whiteling to avoid detecting the same plant again
'''

from time import sleep
from whitelinev1 import whiteline
from fertilizer import do_fertilize

plant_count =0
while True:
	print "============Start========="	
	whiteline(False) 			#Follow white line till plant found
	do_fertilize(plant_count)  #FERILIZE THE PLANT
	plant_count = plant_count + 1 
	sleep(2)
	print "============Done========="
	whiteline(True)				#folow whiteline and clear current plant view
	print "============Clear========="

