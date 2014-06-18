Team 1 : FireBird VI based Feeding Robot 
---------------------------------------------------------

Team
---------
*  ASHOKKUMAR C 	124050006  
*  ROHIT GUPTA 		124050002  
*  SUBHASH KUNNATH 	123050040 

Hardware Requirements
-------------------------------------
? Fire Bird VI robot along with on-board computer 
? Two webcams connected to on-board computer 
? Two DC gear motors 
? DC motor controller (L293D) 
? Atmega32  microcontroller kit including the ISP programmer 
? Two position encoders, one attached with DC motor 
? Retractable arm mechanism 
? Archimedes' screw 
? USB - RS232 converters

Software Requirements
-----------------------------------
? ubuntu 12.04  
? openCV 
? python 
? AVR Studio 
? LPCXpresso v4.1.5_219 

==================================================================================
The objective of our project is to design and build a robot which is capable
of feeding solid fertilizer to the plants present in the greenhouse.

Refer the report to know complete operaion involved.
==================================================================================

How to RUN this project:
------------------------

Step 1: copy the files in folder "FB6 code" and run "RS232_GUI_DEMO" on FB 6
Step 2: copy the files in folder "Atmega32" and burn it on Atmega32 kit
step 3: The folder "On-Board computer" should be copied to the
        on-board computer or host
		----file must be in On-Board computer-----
			main.py			#contains main operations of the project
			connection.py	#Contains connetion details of FB 6 and Atmega32
			fertilizer.py	#contains entire process done during fertilizing
			sensor.py		#contains operations related to sensors
			movements.py	#contains operations related to movements
			opencv_fun.py	#contains operations related to image processing(openc)
			whitelinev1.py	#code to follow whiteline an find plant possition
			template folder # templates required for root detection
	
step 4:run "python main.py" on on-board computer terminal
=====================================================================================
Images
------
px.png	-	image of plant number "x" before fertilizing 
prx.png	-	image of plant number "x" before fertilizing 
======================================================================================

Testing
-------
Testing.py can be used to send simple commend to FB 6
