/*
===============================================================================
 Name        : Hardwareprofile.h
 Author      :
 Version     :
 Copyright   : Copyright (C) 2012
 Description : main definition
===============================================================================
*/

#ifndef HARDWAREPROFILE_H_
#define HARDWAREPROFILE_H_

#define ID 		0x80
#define HW_VER	0x01
#define SW_VER	0x01

#define WHITELINE
#define ULTRASONIC
#define IR_PROXIMITY

#define GYROSCOPE
#define ACCELEROMETER
#define MAGNETOMETER

#define LCD
#define MOTORCONTROLLER
#define BATTERYMONITOR

#define POT
#define LED
#define SWITCH

#define BEAGLE
#define GPS

//#define BLUETOOTH
#define XBEE
//#define WIFI

#define SERVOPOD

#define EXT_ADC

#if (defined(WHITELINE) || defined(ULTRASONIC) || defined(IR_PROXIMITY) || defined(EXT_ADC) || defined(SERVOPOD))
	#define SENSORBOARD
#endif

#if (defined(GYROSCOPE) || defined(ACCELEROMETER) || defined(MAGNETOMETER))
	#define INERTIAL
#endif

#if (defined(BLUETOOTH) || defined(XBEE) || defined(WIFI))
	#define WIRELESS_COMMUNICATION
#endif


#endif /* HARDWARECONFIG_H_ */
