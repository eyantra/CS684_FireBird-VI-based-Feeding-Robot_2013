/*
===============================================================================
 Name        : Gps.h
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains GPS module code header definition.
===============================================================================
*/

#ifndef GPS_H_
#define GPS_H_


void rx_data_store(void);
void readlatitude(void);
void readnorsou(void);
void readlongitude(void);
void readestwst(void);
void readposfix(void);
void readsatellitelock(void);
void readUTC(void);


extern volatile uint8_t data;                                     // to rx current character data
extern volatile uint8_t rx_start,rx_start1,rx_start2,ii; // to avoid re-writing over string
extern volatile uint8_t pos,ggabite;
extern volatile uint8_t rxtxintchngdbit;
extern volatile uint8_t rxtxintchngdbit2;
extern volatile uint8_t dispclrbit;
extern volatile uint8_t usatr3_count;
extern volatile uint8_t rx_string[80];
extern uint8_t temprory[4];
extern const char *msgid[5];
extern volatile uint8_t UTC[10];

#endif /* GPS_H_ */
