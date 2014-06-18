/*
===============================================================================
 Name        : batterymonitor.h
 Author      : Nex Robotics Pvt Ltd
 Version     : 1.0
 Copyright   : Copyright (C) 2012
 Description : This file contains batterymonitor code header definition.
===============================================================================
*/

#ifndef BATTERYMONITOR_H_
#define BATTERYMONITOR_H_

uint32_t Battery_Monitor_WriteReg(uint8_t Addr, uint8_t Data);
uint32_t Battery_Monitor_ReadReg(uint8_t Addr, uint8_t Length);
uint32_t Get_Battery_Status(uint8_t *Data);
extern void Delay(void);

#endif /* BATTERYMONITOR_H_ */
