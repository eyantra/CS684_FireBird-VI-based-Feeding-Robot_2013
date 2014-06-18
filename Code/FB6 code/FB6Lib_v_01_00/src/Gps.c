/*
===============================================================================
 Name        : Gps.c
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains APIs to acquire & display GPS data on LCD screen.
===============================================================================
*/


#include <string.h>
#include "LPC17xx.h"
#include "type.h"
#include "HardwareDefinition.h"
#include "Hardwareprofile.h"
#include "Gps.h"
#include "lcd.h"
#include "uart.h"




volatile uint8_t data;                                     // to rx current character data
volatile uint8_t rx_start,rx_start1,rx_start2,ii; // to avoid re-writing over string
volatile uint8_t pos,ggabite;
volatile uint8_t rxtxintchngdbit = 0;
volatile uint8_t rxtxintchngdbit2 = 0;
volatile uint8_t dispclrbit = 0;
volatile uint8_t usatr3_count = 0;
volatile uint8_t rx_string[80];                         // to store current packet
uint8_t temprory[4];
const char *msgid[5] = {
                    		 "GGA",
							 };

volatile uint8_t rxtxintchngd[17] = "NoCommWith GPSRx";
volatile uint8_t gpsrecv[13] = "GPS Receving";
volatile uint8_t satnlock[15] = "Sat Not Locked";
volatile uint8_t sat[4] = "SAT";
volatile uint8_t fix[4] = "FIX";

volatile uint8_t ggamsg[76];                            // to final store of the GGA received packet

volatile uint8_t latitude[10];							// latitude = ddmm.mmmm
volatile uint8_t norsou[2];								// N/S indicator = N=North or S=South
volatile uint8_t longitude[11];							// longitude = dddmm.mmmm
volatile uint8_t estwst[2]; 							// E/W indicator = E=East or W=West
volatile uint8_t posfix[2];								// positon indicator 0=fix not available or invalid,
											//					 1=GPS SPS mode, fix valid
volatile uint8_t UTC[10];
volatile uint8_t satellitelock[3];						// satellite used


#ifdef GPS

/*===============================================================================
 Function        : readUTC();
 Parameters		 : None
 Description 	 : This function extracts UTC information from GPS packet and
 	 	 	  	   displays it on LCD
 Preconditions	 : uncomment GPS & LCD definitions in Hardwareprofile.h
===============================================================================*/
void readUTC(void)
{
	for(pos=0,ggabite=7;pos<10;pos++,ggabite++)
	{ UTC[pos] = ggamsg[ggabite];}
	UTC[10] = '\0';
	#ifdef LCD
	LCDSetCursorPosition(2,5);
	LCD_WriteStr(6,(uint8_t*)UTC);
	#endif
}


/*===============================================================================
 Function        : readlatitude();
 Parameters		 : None
 Description 	 : This function extracts Latitude information from GPS packet and
 	 	 	  	   displays it on LCD
 Preconditions	 : uncomment GPS & LCD definitions in Hardwareprofile.h
===============================================================================*/
void readlatitude(void)
{
	for(pos=0,ggabite=18;pos<9;pos++,ggabite++)
	{ latitude[pos] = ggamsg[ggabite];}
	latitude[9] = '\0';
	#ifdef LCD
	LCDSetCursorPosition(3,6);
	LCD_WriteStr(9,(uint8_t*)latitude);
	#endif
}


/*===============================================================================
 Function        : readnorsou();
 Parameters		 : None
 Description 	 : This function extracts Latitude cardinal information from GPS packet and
 	 	 	  	   displays it on LCD
 Preconditions	 : uncomment GPS & LCD definitions in Hardwareprofile.h
===============================================================================*/
void readnorsou(void)
{
	norsou[0] = ggamsg[28];
	norsou[1] = '\0';
	#ifdef LCD
	LCDSetCursorPosition(3,15);
	LCD_WriteStr(1,(uint8_t*)norsou);
	#endif
}


/*===============================================================================
 Function        : readlongitude();
 Parameters		 : None
 Description 	 : This function extracts Longitude information from GPS packet and
 	 	 	  	   displays it on LCD
 Preconditions	 : uncomment GPS & LCD definitions in Hardwareprofile.h
===============================================================================*/
void readlongitude(void)
{
	for(pos=0,ggabite=30;pos<10;pos++,ggabite++)
	{ longitude[pos] = ggamsg[ggabite];}
	longitude[10] = '\0';
	#ifdef LCD
	LCDSetCursorPosition(4,5);
	LCD_WriteStr(10,(uint8_t*)longitude);
	#endif
}


/*===============================================================================
 Function        : readestwst();
 Parameters		 : None
 Description 	 : This function extracts Longitude cardinal information from GPS packet and
 	 	 	  	   displays it on LCD
 Preconditions	 : uncomment GPS & LCD definitions in Hardwareprofile.h
===============================================================================*/
void readestwst(void)
{
	estwst[0] = ggamsg[41];
	estwst[1] = '\0';
	#ifdef LCD
	LCDSetCursorPosition(4,15);
	LCD_WriteStr(1,(uint8_t*)estwst);
	#endif
}


/*===============================================================================
 Function        : readposfix();
 Parameters		 : None
 Description 	 : This function extracts position fix information from GPS packet and
 	 	 	  	   displays it on LCD
 	 	 	  	   0 - Invalid
 	 	 	  	   1 - GPS Fix
 	 	 	  	   2 - DGPS Fix
 Preconditions	 : uncomment GPS & LCD definitions in Hardwareprofile.h
===============================================================================*/
void readposfix(void)
{
	posfix[0] = ggamsg[43];
	posfix[1] = '\0';
	#ifdef LCD
	LCDSetCursorPosition(2,19);
	LCD_WriteStr(1,(uint8_t*)posfix);
	#endif
}



/*===============================================================================
 Function        : readsatellitelock();
 Parameters		 : None
 Description 	 : This function extracts no. of satellites information from GPS packet and
 	 	 	  	   displays it on LCD
 Preconditions	 : uncomment GPS & LCD definitions in Hardwareprofile.h
===============================================================================*/
void readsatellitelock(void)
{
	if(ggamsg[46] != ',')
	{
		satellitelock[0] = ggamsg[45];
		satellitelock[1] = ggamsg[46];
	}
	else
	{
		satellitelock[0] = 0x30;
		satellitelock[1] = ggamsg[45];
	}
	satellitelock[2] = '\0';
	#ifdef LCD
	LCDSetCursorPosition(1,19);
	LCD_WriteStr(2,(uint8_t*)satellitelock);
	#endif
}


/*===============================================================================
 Function        : rx_data_store();
 Parameters		 : None
 Description 	 : This function check for valid GPS packet and extracts
 	 	 	 	   useful information from the packet and displays it on LCD screen
 Preconditions	 : uncomment GPS & LCD definitions in Hardwareprofile.h
===============================================================================*/
void rx_data_store(void)
{
	if(gpsPacketRx==1)
	{
		temprory[0] = rx_string[3];
		temprory[1] = rx_string[4];
		temprory[2] = rx_string[5];
		if(!(strcmp((const char *)temprory,(const char *)msgid[0])))
		{
			strncpy((char *)ggamsg,(const char *)rx_string,80);       // copy the GPGGA data from Received string to extract the data//ggamsg[22] = 0x30;                  // store 0 at 22nd location of ggamsg[] array
			readUTC();
			readlatitude();                   // call the function for reading and displaying Latitude
			readnorsou();                     // call the function for reading and displaying North or South
			readlongitude();                  // call the function for reading and displaying Longitude
			readestwst();                     // call the function for reading and displaying East or West
			readposfix();                     // call the function for reading and displaying GPS SPS mode
			readsatellitelock();              // call the function for reading and displaying No. of satellite locked
			rx_start2 = 0;
			dispclrbit = 0;
		}
		gpsPacketRx = 0;
		rx_start2 = 0;                        // enable the next string receiving
	}
}

#endif //GPS

